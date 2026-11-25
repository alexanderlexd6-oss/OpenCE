/*
HOST_SM64_GAME.C

Super Mario 64's engine (libsm64) as host services for the guest.
Step 2 of the "Mario in Halo" experiment: the host functions the guest will
call (declared in guest/runtime/guest_host.h, listed in host_imports.list),
plus a self-test that uses them without the guest.

It stays dormant unless the data folder holds libsm64.so and
baserom.us.z64. The self-test runs once per empty file named sm64_test2
(renamed to sm64_test2.done); its log, and any failure of the functions
below, goes to sm64_game.txt in the data folder.

Units are SM64's own (Mario is about 160 units tall). Only ints, floats in
plain structs and pointers cross to the guest.
*/

#include "host.h"

#include <dlfcn.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* host_main.c */
void host_android_path(int which, char *buffer, uint32_t size);

#define SM64_CACHE_LIBRARY "/data/data/com.halo.decomp/cache/libsm64.so"
#define SM64_MAX_TRIANGLES 1024
/* the Retro64 fork steps at 30 Hz by the clock: ticks closer than this do
nothing, so the host waits out the difference */
#define SM64_TICK_US 34000
/* the atlas built from the ROM, probably 704 x 64 RGBA (the buffer is 1 MB) */
#define SM64_TEXTURE_BUFFER (1 << 20)
#define SM64_TEXTURE_BYTES (704 * 64 * 4)

/* the guest's view (guest_host.h has the same two structures) */
struct host_sm64_input
{
	float stick_x;
	float stick_y;
	float cam_look_x;
	float cam_look_z;
	int button_a;
	int button_b;
	int button_z;
};

struct host_sm64_state
{
	float position[3];
	float velocity[3];
	float face_angle;
	int health;
	unsigned int action;
	unsigned int flags;
	unsigned int triangles;
};

/* the library's, in the order of the Retro64 fork's Java bindings; read and
written through buffers larger than they need */
struct lib_surface
{
	int16_t type;
	int16_t force;
	uint16_t terrain;
	int32_t vertices[3][3];
};

struct lib_inputs
{
	float cam_look_x;
	float cam_look_z;
	float camera_position[3];
	float stick_x;
	float stick_y;
	uint8_t button_a;
	uint8_t button_b;
	uint8_t button_z;
};

struct lib_state
{
	float position[3];
	float velocity[3];
	float face_angle;
	int16_t health;
	uint32_t flags;
	uint32_t action;
	uint32_t current_model;
};

struct lib_geometry
{
	float *position;
	float *normal;
	float *color;
	float *uv;
	uint16_t triangles_used;
};

static pthread_mutex_t sm64_lock = PTHREAD_MUTEX_INITIALIZER;
/* 0: not tried, 1: ready, -1: failed (for good) */
static int sm64_state_of_init;
static char sm64_root[512];
static void *sm64_library;
static uint8_t *sm64_texture;
static struct lib_geometry sm64_geometry_buffers;
static int64_t sm64_last_tick_us;

static void (*lib_global_init)(const uint8_t *, uint8_t *, void (*)(const char *));
static void (*lib_static_surfaces_load)(const struct lib_surface *, uint32_t);
static int (*lib_mario_create)(float, float, float);
static void (*lib_mario_tick)(int, const struct lib_inputs *, struct lib_state *, struct lib_geometry *);
static void (*lib_mario_delete)(int);

static void sm64_log(const char *format, ...)
{
	char text[512];
	char path[640];
	va_list arguments;
	FILE *file;

	va_start(arguments, format);
	vsnprintf(text, sizeof(text), format, arguments);
	va_end(arguments);
	host_logf(HOST_LOG_INFO, "sm64: %s", text);
	if (!sm64_root[0])
		return;
	snprintf(path, sizeof(path), "%s/sm64_game.txt", sm64_root);
	file = fopen(path, "a");
	if (file)
	{
		fprintf(file, "%s\n", text);
		fclose(file);
	}
}

static void sm64_library_message(const char *text)
{
	sm64_log("[libsm64] %s", text);
}

static int64_t sm64_now_us(void)
{
	struct timespec now;

	clock_gettime(CLOCK_MONOTONIC, &now);
	return (int64_t)now.tv_sec * 1000000LL + now.tv_nsec / 1000;
}

static void *sm64_read_file(const char *path, size_t *size)
{
	FILE *file = fopen(path, "rb");
	void *data;
	long length;

	if (!file)
		return NULL;
	fseek(file, 0, SEEK_END);
	length = ftell(file);
	fseek(file, 0, SEEK_SET);
	if (length <= 0)
	{
		fclose(file);
		return NULL;
	}
	data = malloc((size_t)length);
	if (data && fread(data, 1, (size_t)length, file) != (size_t)length)
	{
		free(data);
		data = NULL;
	}
	fclose(file);
	if (data)
		*size = (size_t)length;
	return data;
}

static int sm64_copy_file(const char *from, const char *to)
{
	FILE *in = fopen(from, "rb");
	FILE *out;
	char buffer[65536];
	size_t count;

	if (!in)
		return -1;
	out = fopen(to, "wb");
	if (!out)
	{
		fclose(in);
		return -1;
	}
	while ((count = fread(buffer, 1, sizeof(buffer), in)) > 0)
		fwrite(buffer, 1, count, out);
	fclose(in);
	fclose(out);
	chmod(to, 0600);
	return 0;
}

/* loads the library and the ROM from the data folder, once.
0: ready; -1: failed earlier; -2: the data folder is not chosen yet (try
again); -3: libsm64.so could not be copied; -4: dlopen failed; -5: a
function is missing; -6: the ROM could not be read */
int host_sm64_init(void)
{
	char path[640];
	size_t rom_size = 0;
	uint8_t *rom;
	int result = 0;

	pthread_mutex_lock(&sm64_lock);
	if (sm64_state_of_init != 0)
	{
		result = sm64_state_of_init > 0 ? 0 : -1;
		pthread_mutex_unlock(&sm64_lock);
		return result;
	}
	sm64_root[0] = '\0';
	host_android_path(0, sm64_root, sizeof(sm64_root));
	if (!sm64_root[0])
	{
		pthread_mutex_unlock(&sm64_lock);
		return -2;
	}
	sm64_log("init: data folder %s", sm64_root);

	snprintf(path, sizeof(path), "%s/libsm64.so", sm64_root);
	if (sm64_copy_file(path, SM64_CACHE_LIBRARY) != 0)
	{
		sm64_log("FAILED: cannot copy %s", path);
		result = -3;
		goto fail;
	}
	sm64_library = dlopen(SM64_CACHE_LIBRARY, RTLD_NOW);
	if (!sm64_library)
	{
		sm64_log("FAILED: dlopen: %s", dlerror());
		result = -4;
		goto fail;
	}
	*(void **)&lib_global_init = dlsym(sm64_library, "sm64_global_init");
	*(void **)&lib_static_surfaces_load = dlsym(sm64_library, "sm64_static_surfaces_load");
	*(void **)&lib_mario_create = dlsym(sm64_library, "sm64_mChar_create");
	*(void **)&lib_mario_tick = dlsym(sm64_library, "sm64_mChar_tick");
	*(void **)&lib_mario_delete = dlsym(sm64_library, "sm64_mChar_delete");
	if (!lib_global_init || !lib_static_surfaces_load || !lib_mario_create || !lib_mario_tick || !lib_mario_delete)
	{
		sm64_log("FAILED: the library lacks a function");
		result = -5;
		goto fail;
	}
	snprintf(path, sizeof(path), "%s/baserom.us.z64", sm64_root);
	rom = sm64_read_file(path, &rom_size);
	if (!rom)
	{
		sm64_log("FAILED: cannot read the ROM %s", path);
		result = -6;
		goto fail;
	}
	sm64_log("ROM read, %lu bytes", (unsigned long)rom_size);

	sm64_texture = calloc(1, SM64_TEXTURE_BUFFER);
	sm64_geometry_buffers.position = calloc(1, SM64_MAX_TRIANGLES * 9 * sizeof(float) * 2);
	sm64_geometry_buffers.normal = calloc(1, SM64_MAX_TRIANGLES * 9 * sizeof(float) * 2);
	sm64_geometry_buffers.color = calloc(1, SM64_MAX_TRIANGLES * 9 * sizeof(float) * 2);
	sm64_geometry_buffers.uv = calloc(1, SM64_MAX_TRIANGLES * 6 * sizeof(float) * 2);
	lib_global_init(rom, sm64_texture, sm64_library_message);
	sm64_log("sm64_global_init returned");
	sm64_state_of_init = 1;
	pthread_mutex_unlock(&sm64_lock);
	return 0;

fail:
	sm64_state_of_init = -1;
	pthread_mutex_unlock(&sm64_lock);
	return result;
}

/* a square floor at height y, half_size units to each side (both triangles
face up); replaces any floor loaded before */
void host_sm64_floor(int y, int half_size)
{
	struct lib_surface surfaces[2];
	int s = half_size;
	int index;

	if (sm64_state_of_init <= 0)
		return;
	memset(surfaces, 0, sizeof(surfaces));
	surfaces[0].terrain = 1;
	surfaces[0].vertices[0][0] = -s; surfaces[0].vertices[0][2] = s;
	surfaces[0].vertices[1][0] = s;  surfaces[0].vertices[1][2] = s;
	surfaces[0].vertices[2][0] = s;  surfaces[0].vertices[2][2] = -s;
	surfaces[1].terrain = 1;
	surfaces[1].vertices[0][0] = -s; surfaces[1].vertices[0][2] = s;
	surfaces[1].vertices[1][0] = s;  surfaces[1].vertices[1][2] = -s;
	surfaces[1].vertices[2][0] = -s; surfaces[1].vertices[2][2] = -s;
	for (index = 0; index < 6; index++)
		surfaces[index / 3].vertices[index % 3][1] = y;
	pthread_mutex_lock(&sm64_lock);
	lib_static_surfaces_load(surfaces, 2);
	pthread_mutex_unlock(&sm64_lock);
	sm64_log("floor loaded at y=%d, half size %d", y, half_size);
}

/* Mario's id, or -1 */
int host_sm64_create(int x, int y, int z)
{
	int id;

	if (sm64_state_of_init <= 0)
		return -1;
	pthread_mutex_lock(&sm64_lock);
	id = lib_mario_create((float)x, (float)y, (float)z);
	pthread_mutex_unlock(&sm64_lock);
	sm64_log("Mario created at %d %d %d, id %d", x, y, z, id);
	return id;
}

void host_sm64_delete(int id)
{
	if (sm64_state_of_init <= 0 || id < 0)
		return;
	pthread_mutex_lock(&sm64_lock);
	lib_mario_delete(id);
	pthread_mutex_unlock(&sm64_lock);
}

/* one step, called at the game's 30 ticks a second: waits out the rest of
34 ms since the last one; fills state; the geometry waits for
host_sm64_geometry. 0, or -1 when the engine is not ready */
int host_sm64_tick(int id, const struct host_sm64_input *input, struct host_sm64_state *state)
{
	union { struct lib_inputs fields; char pad[256]; } inputs;
	union { struct lib_state fields; char pad[512]; } engine;
	int64_t elapsed;

	if (sm64_state_of_init <= 0 || id < 0 || !input || !state)
		return -1;
	memset(&inputs, 0, sizeof(inputs));
	memset(&engine, 0, sizeof(engine));
	inputs.fields.cam_look_x = input->cam_look_x;
	inputs.fields.cam_look_z = input->cam_look_z;
	inputs.fields.stick_x = input->stick_x;
	inputs.fields.stick_y = input->stick_y;
	inputs.fields.button_a = input->button_a != 0;
	inputs.fields.button_b = input->button_b != 0;
	inputs.fields.button_z = input->button_z != 0;

	pthread_mutex_lock(&sm64_lock);
	elapsed = sm64_now_us() - sm64_last_tick_us;
	if (elapsed < SM64_TICK_US)
		usleep((useconds_t)(SM64_TICK_US - elapsed));
	sm64_last_tick_us = sm64_now_us();
	lib_mario_tick(id, &inputs.fields, &engine.fields, &sm64_geometry_buffers);
	memcpy(state->position, engine.fields.position, sizeof(state->position));
	memcpy(state->velocity, engine.fields.velocity, sizeof(state->velocity));
	state->face_angle = engine.fields.face_angle;
	state->health = engine.fields.health;
	state->action = engine.fields.action;
	state->flags = engine.fields.flags;
	state->triangles = sm64_geometry_buffers.triangles_used;
	pthread_mutex_unlock(&sm64_lock);
	return 0;
}

/* copies the last tick's triangles: position and normal and color 9 floats a
triangle, uv 6; returns how many triangles (at most capacity) */
int host_sm64_geometry(float *position, float *normal, float *color, float *uv, int capacity)
{
	int count;

	if (sm64_state_of_init <= 0 || !position || !normal || !color || !uv || capacity <= 0)
		return 0;
	pthread_mutex_lock(&sm64_lock);
	count = sm64_geometry_buffers.triangles_used;
	if (count > capacity)
		count = capacity;
	if (count > SM64_MAX_TRIANGLES)
		count = SM64_MAX_TRIANGLES;
	memcpy(position, sm64_geometry_buffers.position, (size_t)count * 9 * sizeof(float));
	memcpy(normal, sm64_geometry_buffers.normal, (size_t)count * 9 * sizeof(float));
	memcpy(color, sm64_geometry_buffers.color, (size_t)count * 9 * sizeof(float));
	memcpy(uv, sm64_geometry_buffers.uv, (size_t)count * 6 * sizeof(float));
	pthread_mutex_unlock(&sm64_lock);
	return count;
}

/* copies the texture atlas (RGBA) into data; returns the bytes copied */
int host_sm64_texture(unsigned char *data, int size)
{
	if (sm64_state_of_init <= 0 || !data || size <= 0)
		return 0;
	if (size > SM64_TEXTURE_BYTES)
		size = SM64_TEXTURE_BYTES;
	memcpy(data, sm64_texture, (size_t)size);
	return size;
}

/* ---------- self-test (no guest): the same functions, as the guest will call them */

static void selftest(const char *root)
{
	char flag_path[640];
	char done_path[640];
	struct host_sm64_input input;
	struct host_sm64_state state;
	float *position, *normal, *color, *uv;
	unsigned char *texture;
	int result, mario, frame, count, index, filled = 0;
	const char *phase;

	snprintf(flag_path, sizeof(flag_path), "%s/sm64_test2", root);
	snprintf(done_path, sizeof(done_path), "%s/sm64_test2.done", root);
	if (access(flag_path, F_OK) != 0)
		return;
	rename(flag_path, done_path);

	result = host_sm64_init();
	sm64_log("selftest: host_sm64_init returned %d", result);
	if (result != 0)
		goto done;
	host_sm64_floor(0, 2000);
	mario = host_sm64_create(0, 500, 0);
	if (mario < 0)
		goto done;
	memset(&input, 0, sizeof(input));
	input.cam_look_z = 1.0f;
	for (frame = 0; frame < 200; frame++)
	{
		if (frame < 60)
		{
			phase = "fall";
			input.stick_y = 0.0f;
			input.button_a = 0;
		}
		else if (frame < 140)
		{
			phase = "walk";
			input.stick_y = 1.0f;
			input.button_a = 0;
		}
		else
		{
			phase = "jump";
			input.stick_y = 1.0f;
			input.button_a = frame < 150;
		}
		memset(&state, 0, sizeof(state));
		/* no pause here: host_sm64_tick paces itself */
		host_sm64_tick(mario, &input, &state);
		if (frame % 20 == 0 || frame == 199)
			sm64_log("%-5s frame %3d pos %8.2f %8.2f %8.2f action 0x%08X triangles %u",
				phase, frame, state.position[0], state.position[1], state.position[2],
				state.action, state.triangles);
	}
	position = calloc(SM64_MAX_TRIANGLES, 9 * sizeof(float));
	normal = calloc(SM64_MAX_TRIANGLES, 9 * sizeof(float));
	color = calloc(SM64_MAX_TRIANGLES, 9 * sizeof(float));
	uv = calloc(SM64_MAX_TRIANGLES, 6 * sizeof(float));
	texture = calloc(1, SM64_TEXTURE_BYTES);
	count = host_sm64_geometry(position, normal, color, uv, SM64_MAX_TRIANGLES);
	sm64_log("selftest: geometry %d triangles; first vertex %.2f %.2f %.2f, uv %.3f %.3f, color %.2f %.2f %.2f",
		count, position[0], position[1], position[2], uv[0], uv[1], color[0], color[1], color[2]);
	count = host_sm64_texture(texture, SM64_TEXTURE_BYTES);
	for (index = 0; index < count; index++)
		filled += texture[index] != 0;
	sm64_log("selftest: texture %d bytes, %d not zero", count, filled);
	host_sm64_delete(mario);
	sm64_log("selftest: RESULT done");
done:
	sm64_log("selftest: end");
}

static void *selftest_thread(void *unused)
{
	char root[512];
	int waited;

	(void)unused;
	for (waited = 0; waited < 120; waited++)
	{
		root[0] = '\0';
		host_android_path(0, root, sizeof(root));
		if (root[0])
		{
			usleep(1500 * 1000);
			host_android_path(0, root, sizeof(root));
			selftest(root);
			return NULL;
		}
		usleep(500 * 1000);
	}
	return NULL;
}

__attribute__((constructor)) static void sm64_game_start(void)
{
	pthread_t thread;

	if (pthread_create(&thread, NULL, selftest_thread, NULL) == 0)
		pthread_detach(thread);
}
