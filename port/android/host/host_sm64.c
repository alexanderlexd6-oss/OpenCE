/*
HOST_SM64.C

Smoke test of libsm64 (Super Mario 64's engine as a library) inside the host.
Step 1 of the "Mario in Halo" experiment: it only checks that the library
loads in the app, accepts the player's own ROM, and that Mario stands, walks
and jumps on a flat floor. It does not touch the game.

It stays dormant unless the data folder (the one with maps/) holds:
  libsm64.so        the ARM64 Android build of libsm64
  baserom.us.z64    the player's own Super Mario 64 (USA) ROM
  sm64_test         an empty file; it is renamed to sm64_test.done when the
                    test starts, so the test runs once per file created

The result goes to sm64_test.txt in the data folder.

No other source file changes: a constructor starts a thread that waits for
host_main.c to choose the data folder.
*/

#include "host.h"

#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

/* host_main.c */
void host_android_path(int which, char *buffer, uint32_t size);

/* The structures of the Retro64 fork of libsm64, in the order of its Java
bindings. They are only read and written through buffers far larger than
they need, so that a layout mistake shows up as odd numbers in the log and
never as a memory overwrite. */

struct sm64_surface
{
	int16_t type;
	int16_t force;
	uint16_t terrain;
	int32_t vertices[3][3];
};

struct sm64_inputs
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

struct sm64_state
{
	float position[3];
	float velocity[3];
	float face_angle;
	int16_t health;
	uint32_t flags;
	uint32_t action;
	uint32_t current_model;
};

struct sm64_geometry
{
	float *position;
	float *normal;
	float *color;
	float *uv;
	uint16_t triangles_used;
};

typedef void (*sm64_debug_print_t)(const char *);

static FILE *report;

static void sm64_debug_print(const char *text)
{
	if (report)
		fprintf(report, "[libsm64] %s\n", text);
}

static void *read_whole_file(const char *path, size_t *size)
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

static int copy_file(const char *from, const char *to)
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

static void log_state(int frame, const char *label, const struct sm64_state *state, const struct sm64_geometry *geometry)
{
	fprintf(report,
		"%-6s frame %3d  pos %9.2f %9.2f %9.2f  vel %7.2f %7.2f %7.2f  action 0x%08X  health %d  triangles %u\n",
		label, frame, state->position[0], state->position[1], state->position[2],
		state->velocity[0], state->velocity[1], state->velocity[2],
		(unsigned)state->action, (int)state->health, (unsigned)geometry->triangles_used);
	fflush(report);
}

static void run_test(const char *root)
{
	char path[640];
	char library_path[640];
	char flag_path[640];
	char done_path[640];
	void *library;
	size_t rom_size = 0;
	uint8_t *rom;
	uint8_t *texture;
	struct sm64_surface floor_surfaces[2];
	/* oversized on purpose (see above) */
	union { struct sm64_inputs fields; char pad[256]; } inputs;
	union { struct sm64_state fields; char pad[512]; } state;
	struct sm64_geometry geometry;
	const int S = 2000;
	int mario;
	int frame;
	const char *phase;

	void (*global_init)(const uint8_t *, uint8_t *, sm64_debug_print_t);
	void (*static_surfaces_load)(const struct sm64_surface *, uint32_t);
	int (*mario_create)(float, float, float);
	void (*mario_tick)(int, const struct sm64_inputs *, struct sm64_state *, struct sm64_geometry *);
	void (*mario_delete)(int);
	void (*global_terminate)(void);

	snprintf(flag_path, sizeof(flag_path), "%s/sm64_test", root);
	snprintf(done_path, sizeof(done_path), "%s/sm64_test.done", root);
	if (access(flag_path, F_OK) != 0)
		return;
	/* renamed first: should the test crash the app, the next start is normal */
	rename(flag_path, done_path);

	snprintf(path, sizeof(path), "%s/sm64_test.txt", root);
	report = fopen(path, "w");
	if (!report)
	{
		host_logf(HOST_LOG_WARN, "sm64 test: cannot write %s", path);
		return;
	}
	fprintf(report, "sm64 test, step 1 (data folder %s)\n", root);
	fflush(report);

	/* the library is run from the app's private folder: the shared storage
	cannot hold code the app loads */
	snprintf(path, sizeof(path), "%s/libsm64.so", root);
	snprintf(library_path, sizeof(library_path), "/data/data/com.halo.decomp/cache/libsm64.so");
	if (copy_file(path, library_path) != 0)
	{
		fprintf(report, "FAILED: cannot copy %s to %s (%s)\n", path, library_path, strerror(errno));
		goto done;
	}
	library = dlopen(library_path, RTLD_NOW);
	if (!library)
	{
		fprintf(report, "FAILED: dlopen: %s\n", dlerror());
		goto done;
	}
	fprintf(report, "libsm64.so loaded\n");
	fflush(report);

	*(void **)&global_init = dlsym(library, "sm64_global_init");
	*(void **)&static_surfaces_load = dlsym(library, "sm64_static_surfaces_load");
	*(void **)&mario_create = dlsym(library, "sm64_mChar_create");
	*(void **)&mario_tick = dlsym(library, "sm64_mChar_tick");
	*(void **)&mario_delete = dlsym(library, "sm64_mChar_delete");
	*(void **)&global_terminate = dlsym(library, "sm64_global_terminate");
	if (!global_init || !static_surfaces_load || !mario_create || !mario_tick || !mario_delete)
	{
		fprintf(report, "FAILED: the library lacks a function the test needs\n");
		goto done;
	}

	snprintf(path, sizeof(path), "%s/baserom.us.z64", root);
	rom = read_whole_file(path, &rom_size);
	if (!rom)
	{
		fprintf(report, "FAILED: cannot read the ROM %s\n", path);
		goto done;
	}
	fprintf(report, "ROM read, %lu bytes (a Super Mario 64 USA .z64 has 8388608)\n", (unsigned long)rom_size);
	fflush(report);

	texture = calloc(1, 1 << 20);
	global_init(rom, texture, sm64_debug_print);
	fprintf(report, "sm64_global_init returned\n");
	fflush(report);

	/* a square floor: both triangles face up (y of the normal > 0) */
	memset(floor_surfaces, 0, sizeof(floor_surfaces));
	floor_surfaces[0].terrain = 1;
	floor_surfaces[0].vertices[0][0] = -S; floor_surfaces[0].vertices[0][2] = S;
	floor_surfaces[0].vertices[1][0] = S;  floor_surfaces[0].vertices[1][2] = S;
	floor_surfaces[0].vertices[2][0] = S;  floor_surfaces[0].vertices[2][2] = -S;
	floor_surfaces[1].terrain = 1;
	floor_surfaces[1].vertices[0][0] = -S; floor_surfaces[1].vertices[0][2] = S;
	floor_surfaces[1].vertices[1][0] = S;  floor_surfaces[1].vertices[1][2] = -S;
	floor_surfaces[1].vertices[2][0] = -S; floor_surfaces[1].vertices[2][2] = -S;
	static_surfaces_load(floor_surfaces, 2);
	fprintf(report, "floor loaded\n");
	fflush(report);

	memset(&geometry, 0, sizeof(geometry));
	geometry.position = calloc(1, 1024 * 9 * sizeof(float) * 2);
	geometry.normal = calloc(1, 1024 * 9 * sizeof(float) * 2);
	geometry.color = calloc(1, 1024 * 9 * sizeof(float) * 2);
	geometry.uv = calloc(1, 1024 * 6 * sizeof(float) * 2);

	mario = mario_create(0.0f, 500.0f, 0.0f);
	fprintf(report, "Mario created, id %d\n", mario);
	fflush(report);
	if (mario < 0)
	{
		fprintf(report, "FAILED: Mario was not created (negative id)\n");
		goto done;
	}

	memset(&inputs, 0, sizeof(inputs));
	memset(&state, 0, sizeof(state));
	inputs.fields.cam_look_x = 0.0f;
	inputs.fields.cam_look_z = 1.0f;
	for (frame = 0; frame < 200; frame++)
	{
		if (frame < 60)
		{
			phase = "fall";
			inputs.fields.stick_y = 0.0f;
			inputs.fields.button_a = 0;
		}
		else if (frame < 140)
		{
			phase = "walk";
			inputs.fields.stick_y = 1.0f;
			inputs.fields.button_a = 0;
		}
		else
		{
			phase = "jump";
			inputs.fields.stick_y = 1.0f;
			inputs.fields.button_a = frame < 150;
		}
		mario_tick(mario, &inputs.fields, &state.fields, &geometry);
		if (frame % 10 == 0 || frame == 199)
			log_state(frame, phase, &state.fields, &geometry);
	}
	fprintf(report, "RESULT: the engine ran 200 frames without crashing\n");
	mario_delete(mario);
	if (global_terminate)
		global_terminate();

done:
	fprintf(report, "end of test\n");
	fclose(report);
	report = NULL;
}

static void *test_thread(void *unused)
{
	char root[512];
	int waited;

	(void)unused;
	/* host_main.c fixes the data folder a moment after the library loads */
	for (waited = 0; waited < 120; waited++)
	{
		root[0] = '\0';
		host_android_path(0, root, sizeof(root));
		if (root[0])
		{
			/* let host_main.c finish writing the path */
			usleep(1000 * 1000);
			host_android_path(0, root, sizeof(root));
			run_test(root);
			return NULL;
		}
		usleep(500 * 1000);
	}
	return NULL;
}

__attribute__((constructor)) static void sm64_test_start(void)
{
	pthread_t thread;

	if (pthread_create(&thread, NULL, test_thread, NULL) == 0)
		pthread_detach(thread);
}
