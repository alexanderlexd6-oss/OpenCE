/* sm64_halo: puente entre libsm64 y el motor de Halo (OpenCE).
 * Coordenadas Halo: X adelante, Y izquierda, Z arriba (mundo, "wu").
 * Todo lo que entra/sale de este modulo esta en coordenadas Halo. */
#ifndef SM64_HALO_H
#define SM64_HALO_H
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { float x, y, z; } HaloVec3;

typedef struct {
    float stick_x, stick_y;      /* -1..1. x+ = derecha, y+ = adelante (joystick tactil) */
    float cam_yaw_delta;         /* radianes este tick (arrastre tactil / stick derecho) */
    float cam_pitch_delta;       /* radianes este tick */
    bool  jump, punch, crouch;   /* A, B, Z */
    bool  cam_reset;             /* "camara detras de Mario" (boton L en el original) */
} Sm64HaloInput;

typedef struct {
    HaloVec3 pos, forward, up;
    float    fov_deg;            /* FOV horizontal sugerido */
} Sm64HaloCamera;

typedef struct { float pos[3], normal[3], color[3], uv[2]; } Sm64HaloVertex;

/* El motor entrega triangulos de la BSP de colision a esta funcion, uno por uno. */
typedef void (*Sm64HaloTriSink)(void *sink_ud, const HaloVec3 v[3],
                                const HaloVec3 *plane_normal /* puede ser NULL */);

typedef struct {
    /* OBLIGATORIO: llama a sink() por cada triangulo de colision (suelo/pared/techo)
     * que toque la esfera (center, radius_wu). Una consulta espacial, no todo el mapa. */
    void (*collect_triangles)(void *ud, HaloVec3 center, float radius_wu,
                              Sm64HaloTriSink sink, void *sink_ud);
    /* Opcional: rayo contra el mundo, para que la camara no atraviese paredes. */
    bool (*raycast)(void *ud, HaloVec3 from, HaloVec3 to, HaloVec3 *hit);
    /* Opcional: mueve el biped de Halo a donde esta Mario (enemigos, triggers, items). */
    void (*set_player_position)(void *ud, HaloVec3 feet, float yaw_rad);
    void *ud;
} Sm64HaloHost;

bool  sm64_halo_init(const char *rom_path, const Sm64HaloHost *host);
void  sm64_halo_shutdown(void);
bool  sm64_halo_spawn(HaloVec3 feet_pos);

/* Llamar UNA vez por tick de simulacion (30 Hz). */
void  sm64_halo_tick(const Sm64HaloInput *in);

/* Llamar por cada frame de render; alpha = 0..1 entre el tick anterior y el actual. */
void  sm64_halo_get_camera(float alpha, Sm64HaloCamera *out);
const Sm64HaloVertex *sm64_halo_get_mesh(float alpha, int *num_vertices);
const uint8_t *sm64_halo_texture(int *w, int *h);   /* RGBA8 */

HaloVec3 sm64_halo_get_position(void);
void  sm64_halo_set_camera_distance(float wu);

/* Render GLES3 opcional (sm64_halo_gl.c). Llamar tras dibujar el mundo. */
bool  sm64_halo_gl_init(void);
void  sm64_halo_gl_shutdown(void);
void  sm64_halo_gl_draw(const Sm64HaloCamera *cam, float aspect,
                        float znear, float zfar, float alpha);

#ifdef __cplusplus
}
#endif
#endif
