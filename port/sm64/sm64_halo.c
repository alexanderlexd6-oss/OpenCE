#include "sm64_halo.h"
#include "libsm64.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Ajustes ---------------------------------------------------------- */
#ifndef SM64_HALO_UNITS_PER_WU
#define SM64_HALO_UNITS_PER_WU 228.0f  /* Mario ~160 u ~= 0.70 wu (altura del Jefe Maestro) */
#endif
/* Calibracion de signos (ver INTEGRACION.md, paso "calibrar controles"). */
#ifndef SM64_HALO_CAMLOOK_SIGN
#define SM64_HALO_CAMLOOK_SIGN 1.0f
#endif
#ifndef SM64_HALO_STICKX_SIGN
#define SM64_HALO_STICKX_SIGN 1.0f
#endif

#define SURF_RADIUS_WU    22.0f  /* 22*228 = ~5000 u, cabe en la rejilla de +-8192 de SM64 */
#define SURF_RELOAD_WU     8.0f  /* recargar/recentrar al alejarse tanto del centro */
#define SURF_MAX_EDGE_WU   4.0f  /* triangulos mas largos se subdividen */
#define SURF_MAX       40000
#define CAM_FOCUS_H_WU     0.55f
#define CAM_DEFAULT_DIST   4.4f  /* 1000 u de SM64 */
#define CAM_FOV_DEG       70.0f
#define PI_F 3.14159265358979f

static const float S = SM64_HALO_UNITS_PER_WU;

/* ---- Utilidades ------------------------------------------------------- */
static HaloVec3 hv(float x, float y, float z) { HaloVec3 r = {x, y, z}; return r; }
static HaloVec3 hsub(HaloVec3 a, HaloVec3 b) { return hv(a.x-b.x, a.y-b.y, a.z-b.z); }
static HaloVec3 hlerp(HaloVec3 a, HaloVec3 b, float t) {
    return hv(a.x+(b.x-a.x)*t, a.y+(b.y-a.y)*t, a.z+(b.z-a.z)*t);
}
static float hlen(HaloVec3 a) { return sqrtf(a.x*a.x + a.y*a.y + a.z*a.z); }
static float hdist2(HaloVec3 a, HaloVec3 b) { HaloVec3 d = hsub(a, b); return d.x*d.x+d.y*d.y+d.z*d.z; }
static float wrap_pi(float a) { while (a > PI_F) a -= 2*PI_F; while (a < -PI_F) a += 2*PI_F; return a; }

typedef struct {
    HaloVec3 pos, focus;       /* modelo de camara (sin colision) */
    HaloVec3 out, out_focus;   /* camara final (con colision) */
    HaloVec3 prev_out, prev_focus;
    float dist, pitch;
} Cam;

static struct {
    bool ready, spawned, have_prev;
    int32_t id;
    Sm64HaloHost host;
    uint8_t *rom;
    uint8_t  texture[4 * SM64_TEXTURE_WIDTH * SM64_TEXTURE_HEIGHT];

    HaloVec3 origin, load_center;   /* origin: punto Halo que equivale a (0,0,0) de SM64 */
    HaloVec3 pos; float yaw;

    float gp[9*SM64_GEO_MAX_TRIANGLES], gn[9*SM64_GEO_MAX_TRIANGLES];
    float gc[9*SM64_GEO_MAX_TRIANGLES], gu[6*SM64_GEO_MAX_TRIANGLES];
    Sm64HaloVertex cur[3*SM64_GEO_MAX_TRIANGLES], prev[3*SM64_GEO_MAX_TRIANGLES], blend[3*SM64_GEO_MAX_TRIANGLES];
    int ncur, nprev;

    struct SM64Surface *surf; uint32_t nsurf; bool surf_overflow;
    Cam cam;
} G;

/* ---- Conversion de coordenadas (rotacion propia: conserva el winding) --- */
/* sm64.x = -halo.y ; sm64.y = halo.z ; sm64.z = -halo.x */
static void to_sm64(HaloVec3 h, float o[3]) {
    HaloVec3 d = hsub(h, G.origin);
    o[0] = -d.y * S; o[1] = d.z * S; o[2] = -d.x * S;
}
static HaloVec3 from_sm64(const float s[3]) {
    return hv(G.origin.x - s[2]/S, G.origin.y - s[0]/S, G.origin.z + s[1]/S);
}
static HaloVec3 dir_from_sm64(const float s[3]) { return hv(-s[2], -s[0], s[1]); }

/* ---- Superficies de colision ------------------------------------------ */
typedef struct { HaloVec3 center; } SinkCtx;

static float axis_gap(float c, float lo, float hi) { return c < lo ? lo-c : (c > hi ? c-hi : 0.0f); }
static bool aabb_hits(HaloVec3 a, HaloVec3 b, HaloVec3 c, HaloVec3 ctr, float r) {
    float gx = axis_gap(ctr.x, fminf(fminf(a.x,b.x),c.x), fmaxf(fmaxf(a.x,b.x),c.x));
    float gy = axis_gap(ctr.y, fminf(fminf(a.y,b.y),c.y), fmaxf(fmaxf(a.y,b.y),c.y));
    float gz = axis_gap(ctr.z, fminf(fminf(a.z,b.z),c.z), fmaxf(fmaxf(a.z,b.z),c.z));
    return gx*gx + gy*gy + gz*gz <= r*r;
}

static void push_surface(HaloVec3 a, HaloVec3 b, HaloVec3 c, const HaloVec3 *n) {
    if (G.nsurf >= SURF_MAX) { G.surf_overflow = true; return; }
    float v[3][3];
    to_sm64(a, v[0]); to_sm64(b, v[1]); to_sm64(c, v[2]);
    if (n) { /* orientar el triangulo hacia la normal del plano de Halo */
        float e1[3] = {v[1][0]-v[0][0], v[1][1]-v[0][1], v[1][2]-v[0][2]};
        float e2[3] = {v[2][0]-v[0][0], v[2][1]-v[0][1], v[2][2]-v[0][2]};
        float cr[3] = {e1[1]*e2[2]-e1[2]*e2[1], e1[2]*e2[0]-e1[0]*e2[2], e1[0]*e2[1]-e1[1]*e2[0]};
        float nn[3] = {-n->y, n->z, -n->x};
        if (cr[0]*nn[0] + cr[1]*nn[1] + cr[2]*nn[2] < 0) {
            for (int i = 0; i < 3; i++) { float t = v[1][i]; v[1][i] = v[2][i]; v[2][i] = t; }
        }
    }
    struct SM64Surface *s = &G.surf[G.nsurf];
    memset(s, 0, sizeof *s);
    s->type = 0; /* SURFACE_DEFAULT. TODO: mapear material de Halo (hielo -> resbaladizo, etc.) */
    for (int i = 0; i < 3; i++) for (int k = 0; k < 3; k++) s->vertices[i][k] = (int16_t)lrintf(v[i][k]);
    /* descartar degenerados tras redondear */
    int ax = s->vertices[1][0]-s->vertices[0][0], ay = s->vertices[1][1]-s->vertices[0][1], az = s->vertices[1][2]-s->vertices[0][2];
    int bx = s->vertices[2][0]-s->vertices[0][0], by = s->vertices[2][1]-s->vertices[0][1], bz = s->vertices[2][2]-s->vertices[0][2];
    long cx = (long)ay*bz-(long)az*by, cy = (long)az*bx-(long)ax*bz, cz = (long)ax*by-(long)ay*bx;
    if (cx == 0 && cy == 0 && cz == 0) return;
    G.nsurf++;
}

static void emit_tri(const SinkCtx *s, HaloVec3 a, HaloVec3 b, HaloVec3 c, const HaloVec3 *n, int depth) {
    if (!aabb_hits(a, b, c, s->center, SURF_RADIUS_WU + SURF_MAX_EDGE_WU)) return;
    float ab = hdist2(a, b), bc = hdist2(b, c), ca = hdist2(c, a);
    float m = fmaxf(ab, fmaxf(bc, ca));
    if (m > SURF_MAX_EDGE_WU*SURF_MAX_EDGE_WU && depth < 12) {
        if (ab >= bc && ab >= ca) { HaloVec3 p = hlerp(a, b, .5f); emit_tri(s,a,p,c,n,depth+1); emit_tri(s,p,b,c,n,depth+1); }
        else if (bc >= ca)        { HaloVec3 p = hlerp(b, c, .5f); emit_tri(s,a,b,p,n,depth+1); emit_tri(s,a,p,c,n,depth+1); }
        else                      { HaloVec3 p = hlerp(c, a, .5f); emit_tri(s,a,b,p,n,depth+1); emit_tri(s,p,b,c,n,depth+1); }
        return;
    }
    push_surface(a, b, c, n);
}

static void sink_cb(void *ud, const HaloVec3 v[3], const HaloVec3 *n) {
    emit_tri((const SinkCtx *)ud, v[0], v[1], v[2], n, 0);
}

static void reload_surfaces(HaloVec3 center) {
    G.origin = center; G.load_center = center; G.nsurf = 0; G.surf_overflow = false;
    SinkCtx ctx = { center };
    G.host.collect_triangles(G.host.ud, center, SURF_RADIUS_WU, sink_cb, &ctx);
    if (G.surf_overflow) fprintf(stderr, "[sm64_halo] aviso: demasiadas superficies, recortadas a %d\n", SURF_MAX);
    sm64_static_surfaces_load(G.surf, G.nsurf);
}

/* ---- API --------------------------------------------------------------- */
bool sm64_halo_init(const char *rom_path, const Sm64HaloHost *host) {
    if (G.ready) return true;
    if (!host || !host->collect_triangles) return false;
    FILE *f = fopen(rom_path, "rb");
    if (!f) { fprintf(stderr, "[sm64_halo] no puedo abrir la ROM: %s\n", rom_path); return false; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    G.rom = (uint8_t *)malloc((size_t)n);
    if (!G.rom || fread(G.rom, 1, (size_t)n, f) != (size_t)n) { fclose(f); free(G.rom); G.rom = NULL; return false; }
    fclose(f);
    G.surf = (struct SM64Surface *)calloc(SURF_MAX, sizeof *G.surf);
    if (!G.surf) return false;
    sm64_global_init(G.rom, G.texture);   /* la ROM se conserva hasta shutdown */
    G.host = *host; G.id = -1;
    G.cam.dist = CAM_DEFAULT_DIST; G.cam.pitch = 0.28f;
    G.ready = true;
    return true;
}

void sm64_halo_shutdown(void) {
    if (!G.ready) return;
    if (G.id >= 0) sm64_mario_delete(G.id);
    sm64_global_terminate();
    free(G.surf); free(G.rom);
    memset(&G, 0, sizeof G);
}

bool sm64_halo_spawn(HaloVec3 feet) {
    if (!G.ready) return false;
    if (G.id >= 0) { sm64_mario_delete(G.id); G.id = -1; }
    reload_surfaces(feet);
    G.id = sm64_mario_create(0.0f, 0.2f * S, 0.0f);
    if (G.id < 0) { fprintf(stderr, "[sm64_halo] sm64_mario_create fallo: no hay suelo bajo el punto de aparicion\n"); return false; }
    G.pos = feet; G.yaw = 0; G.spawned = true; G.have_prev = false; G.ncur = G.nprev = 0;
    /* camara inicial: detras de Mario (-X de Halo) */
    G.cam.focus = hv(feet.x, feet.y, feet.z + CAM_FOCUS_H_WU);
    G.cam.pos = hv(feet.x - G.cam.dist, feet.y, feet.z + G.cam.dist * tanf(G.cam.pitch));
    G.cam.out = G.cam.pos; G.cam.out_focus = G.cam.focus;
    G.cam.prev_out = G.cam.out; G.cam.prev_focus = G.cam.out_focus;
    return true;
}

static void cam_update(const Sm64HaloInput *in) {
    Cam *c = &G.cam;
    c->prev_out = c->out; c->prev_focus = c->out_focus;

    HaloVec3 ft = hv(G.pos.x, G.pos.y, G.pos.z + CAM_FOCUS_H_WU);
    c->focus.x += (ft.x - c->focus.x) * 0.6f;
    c->focus.y += (ft.y - c->focus.y) * 0.6f;
    c->focus.z += (ft.z - c->focus.z) * 0.2f;   /* la altura se suaviza mas (sin rebotes) */

    float hx = c->pos.x - c->focus.x, hy = c->pos.y - c->focus.y;
    float len = sqrtf(hx*hx + hy*hy);
    float a = (len < 1e-4f) ? atan2f(-sinf(G.yaw), -cosf(G.yaw)) : atan2f(hy, hx);

    a += in->cam_yaw_delta;
    if (in->cam_reset) a += wrap_pi(atan2f(-sinf(G.yaw), -cosf(G.yaw)) - a) * 0.25f;
    c->pitch += in->cam_pitch_delta;
    if (c->pitch < 0.05f) c->pitch = 0.05f;
    if (c->pitch > 0.90f) c->pitch = 0.90f;

    /* Estilo Lakitu: la camara se "arrastra" si Mario se aleja y se aparta si se acerca. */
    if (len > c->dist)            len += (c->dist - len) * 0.35f;
    else if (len < c->dist*0.7f)  len = c->dist * 0.7f;

    c->pos.x = c->focus.x + cosf(a) * len;
    c->pos.y = c->focus.y + sinf(a) * len;
    c->pos.z = c->focus.z + len * tanf(c->pitch);

    c->out = c->pos; c->out_focus = c->focus;
    if (G.host.raycast) {
        HaloVec3 hit;
        if (G.host.raycast(G.host.ud, c->focus, c->pos, &hit))
            c->out = hlerp(hit, c->focus, 0.05f);   /* un poco delante de la pared */
    }
}

void sm64_halo_tick(const Sm64HaloInput *in) {
    if (!G.ready || !G.spawned) return;

    /* Recentrar el origen y recargar colision cuando Mario se aleja del centro cargado. */
    if (hlen(hsub(G.pos, G.load_center)) > SURF_RELOAD_WU) {
        reload_surfaces(G.pos);
        sm64_set_mario_position(G.id, 0.0f, 0.0f, 0.0f);
    }

    /* Para interpolar: lo anterior pasa a "prev" (ya esta en coordenadas Halo). */
    memcpy(G.prev, G.cur, (size_t)G.ncur * sizeof(Sm64HaloVertex));
    G.nprev = G.ncur; G.have_prev = G.nprev > 0;

    /* Direccion camara -> Mario (horizontal), en coordenadas SM64. */
    HaloVec3 d = hsub(G.pos, G.cam.out);
    float lx = -d.y, lz = -d.x, ll = sqrtf(lx*lx + lz*lz);
    if (ll < 1e-5f) { lx = 0; lz = 1; ll = 1; }

    struct SM64MarioInputs si; memset(&si, 0, sizeof si);
    si.camLookX = SM64_HALO_CAMLOOK_SIGN * lx / ll;
    si.camLookZ = SM64_HALO_CAMLOOK_SIGN * lz / ll;
    si.stickX   = SM64_HALO_STICKX_SIGN * in->stick_x;
    si.stickY   = in->stick_y;
    si.buttonA  = in->jump; si.buttonB = in->punch; si.buttonZ = in->crouch;

    struct SM64MarioState st; memset(&st, 0, sizeof st);
    struct SM64MarioGeometryBuffers geo;
    geo.position = G.gp; geo.normal = G.gn; geo.color = G.gc; geo.uv = G.gu; geo.numTrianglesUsed = 0;
    sm64_mario_tick(G.id, &si, &st, &geo);

    G.pos = from_sm64(st.position);
    G.yaw = st.faceAngle - PI_F;     /* yaw de Halo (0 = +X, antihorario) */

    int nv = geo.numTrianglesUsed * 3;
    if (nv > 3*SM64_GEO_MAX_TRIANGLES) nv = 3*SM64_GEO_MAX_TRIANGLES;
    for (int i = 0; i < nv; i++) {
        float p[3] = {G.gp[i*3], G.gp[i*3+1], G.gp[i*3+2]};
        float n[3] = {G.gn[i*3], G.gn[i*3+1], G.gn[i*3+2]};
        HaloVec3 hp = from_sm64(p), hn = dir_from_sm64(n);
        Sm64HaloVertex *v = &G.cur[i];
        v->pos[0] = hp.x; v->pos[1] = hp.y; v->pos[2] = hp.z;
        v->normal[0] = hn.x; v->normal[1] = hn.y; v->normal[2] = hn.z;
        memcpy(v->color, &G.gc[i*3], sizeof v->color);
        memcpy(v->uv, &G.gu[i*2], sizeof v->uv);
    }
    G.ncur = nv;

    cam_update(in);
    if (G.host.set_player_position) G.host.set_player_position(G.host.ud, G.pos, G.yaw);
}

void sm64_halo_get_camera(float alpha, Sm64HaloCamera *o) {
    HaloVec3 p = hlerp(G.cam.prev_out, G.cam.out, alpha);
    HaloVec3 f = hlerp(G.cam.prev_focus, G.cam.out_focus, alpha);
    HaloVec3 d = hsub(f, p); float l = hlen(d);
    o->pos = p;
    o->forward = l > 1e-6f ? hv(d.x/l, d.y/l, d.z/l) : hv(1, 0, 0);
    o->up = hv(0, 0, 1);
    o->fov_deg = CAM_FOV_DEG;
}

const Sm64HaloVertex *sm64_halo_get_mesh(float alpha, int *n) {
    *n = G.ncur;
    if (!G.have_prev || G.nprev != G.ncur || G.ncur == 0) return G.cur;
    float dx = G.cur[0].pos[0]-G.prev[0].pos[0], dy = G.cur[0].pos[1]-G.prev[0].pos[1], dz = G.cur[0].pos[2]-G.prev[0].pos[2];
    if (dx*dx + dy*dy + dz*dz > 1.0f) return G.cur;   /* teletransporte: no interpolar */
    for (int i = 0; i < G.ncur; i++) {
        G.blend[i] = G.cur[i];
        for (int k = 0; k < 3; k++) G.blend[i].pos[k] = G.prev[i].pos[k] + (G.cur[i].pos[k]-G.prev[i].pos[k]) * alpha;
    }
    return G.blend;
}

const uint8_t *sm64_halo_texture(int *w, int *h) { *w = SM64_TEXTURE_WIDTH; *h = SM64_TEXTURE_HEIGHT; return G.texture; }
HaloVec3 sm64_halo_get_position(void) { return G.pos; }
void sm64_halo_set_camera_distance(float wu) { if (wu > 1.0f && wu < 30.0f) G.cam.dist = wu; }
