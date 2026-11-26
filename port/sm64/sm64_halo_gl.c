/* Dibujo de Mario con OpenGL ES 3. Guarda y restaura el estado GL que toca,
 * porque el motor de OpenCE tiene su propia capa sobre GL. */
#include "sm64_halo.h"
#include <GLES3/gl3.h>
#include <math.h>
#include <string.h>

static GLuint prog, vao, vbo, tex;
static GLint uV, uP, uTex;

static const char *VS =
"#version 300 es\n"
"layout(location=0) in vec3 aPos; layout(location=1) in vec3 aNrm;\n"
"layout(location=2) in vec3 aCol; layout(location=3) in vec2 aUV;\n"
"uniform mat4 uV, uP; out vec3 vCol; out vec2 vUV;\n"
"void main(){ vCol=aCol; vUV=aUV; gl_Position=uP*uV*vec4(aPos,1.0); }\n";
static const char *FS =
"#version 300 es\nprecision mediump float;\n"
"in vec3 vCol; in vec2 vUV; uniform sampler2D uTex; out vec4 o;\n"
"void main(){ vec4 t=texture(uTex,vUV); if(t.a<0.5) discard; o=vec4(vCol*t.rgb,1.0); }\n";

static GLuint compile(GLenum type, const char *src) {
    GLuint s = glCreateShader(type); glShaderSource(s, 1, &src, NULL); glCompileShader(s);
    GLint ok = 0; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) { glDeleteShader(s); return 0; }
    return s;
}

bool sm64_halo_gl_init(void) {
    GLuint vs = compile(GL_VERTEX_SHADER, VS), fs = compile(GL_FRAGMENT_SHADER, FS);
    if (!vs || !fs) return false;
    prog = glCreateProgram(); glAttachShader(prog, vs); glAttachShader(prog, fs); glLinkProgram(prog);
    glDeleteShader(vs); glDeleteShader(fs);
    GLint ok = 0; glGetProgramiv(prog, GL_LINK_STATUS, &ok);
    if (!ok) return false;
    uV = glGetUniformLocation(prog, "uV"); uP = glGetUniformLocation(prog, "uP"); uTex = glGetUniformLocation(prog, "uTex");

    glGenVertexArrays(1, &vao); glGenBuffers(1, &vbo);
    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    const GLsizei stride = sizeof(Sm64HaloVertex);
    glEnableVertexAttribArray(0); glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(Sm64HaloVertex, pos));
    glEnableVertexAttribArray(1); glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(Sm64HaloVertex, normal));
    glEnableVertexAttribArray(2); glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(Sm64HaloVertex, color));
    glEnableVertexAttribArray(3); glVertexAttribPointer(3, 2, GL_FLOAT, GL_FALSE, stride, (void *)offsetof(Sm64HaloVertex, uv));
    glBindVertexArray(0); glBindBuffer(GL_ARRAY_BUFFER, 0);

    int w, h; const uint8_t *px = sm64_halo_texture(&w, &h);
    glGenTextures(1, &tex); glBindTexture(GL_TEXTURE_2D, tex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, px);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
    return true;
}

void sm64_halo_gl_shutdown(void) {
    if (tex) glDeleteTextures(1, &tex);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (vao) glDeleteVertexArrays(1, &vao);
    if (prog) glDeleteProgram(prog);
    prog = vao = vbo = tex = 0;
}

/* Matrices en columna-mayor (formato GL). Mundo Halo: Z arriba. */
static void look_at(float m[16], const float e[3], const float f[3], const float up[3]) {
    float s[3] = { f[1]*up[2]-f[2]*up[1], f[2]*up[0]-f[0]*up[2], f[0]*up[1]-f[1]*up[0] };
    float sl = sqrtf(s[0]*s[0]+s[1]*s[1]+s[2]*s[2]); if (sl < 1e-6f) sl = 1;
    for (int i = 0; i < 3; i++) s[i] /= sl;
    float u[3] = { s[1]*f[2]-s[2]*f[1], s[2]*f[0]-s[0]*f[2], s[0]*f[1]-s[1]*f[0] };
    memset(m, 0, 16*sizeof(float));
    m[0]=s[0]; m[4]=s[1]; m[8]=s[2];
    m[1]=u[0]; m[5]=u[1]; m[9]=u[2];
    m[2]=-f[0]; m[6]=-f[1]; m[10]=-f[2];
    m[12]=-(s[0]*e[0]+s[1]*e[1]+s[2]*e[2]);
    m[13]=-(u[0]*e[0]+u[1]*e[1]+u[2]*e[2]);
    m[14]= (f[0]*e[0]+f[1]*e[1]+f[2]*e[2]);
    m[15]=1;
}
static void perspective(float m[16], float fovy, float aspect, float n, float f) {
    float t = 1.0f / tanf(fovy * 0.5f);
    memset(m, 0, 16*sizeof(float));
    m[0]=t/aspect; m[5]=t; m[10]=(f+n)/(n-f); m[11]=-1; m[14]=2*f*n/(n-f);
}

void sm64_halo_gl_draw(const Sm64HaloCamera *cam, float aspect, float znear, float zfar, float alpha) {
    if (!prog) return;
    int n = 0; const Sm64HaloVertex *mesh = sm64_halo_get_mesh(alpha, &n);
    if (n <= 0) return;

    /* guardar estado */
    GLint oProg, oVao, oArr, oTex, oAct, oFunc, oFront; GLboolean oMask;
    GLboolean oDepth = glIsEnabled(GL_DEPTH_TEST), oCull = glIsEnabled(GL_CULL_FACE), oBlend = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_CURRENT_PROGRAM, &oProg); glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &oVao);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &oArr); glGetIntegerv(GL_ACTIVE_TEXTURE, &oAct);
    glActiveTexture(GL_TEXTURE0); glGetIntegerv(GL_TEXTURE_BINDING_2D, &oTex);
    glGetIntegerv(GL_DEPTH_FUNC, &oFunc); glGetBooleanv(GL_DEPTH_WRITEMASK, &oMask);
    glGetIntegerv(GL_FRONT_FACE, &oFront);

    float fh = cam->fov_deg * 3.14159265f / 180.0f;               /* FOV horizontal -> vertical */
    float fovy = 2.0f * atanf(tanf(fh * 0.5f) / aspect);
    float V[16], P[16], eye[3] = {cam->pos.x, cam->pos.y, cam->pos.z};
    float fwd[3] = {cam->forward.x, cam->forward.y, cam->forward.z}, up[3] = {cam->up.x, cam->up.y, cam->up.z};
    look_at(V, eye, fwd, up); perspective(P, fovy, aspect, znear, zfar);

    glUseProgram(prog);
    glUniformMatrix4fv(uV, 1, GL_FALSE, V); glUniformMatrix4fv(uP, 1, GL_FALSE, P);
    glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D, tex); glUniform1i(uTex, 0);
    glEnable(GL_DEPTH_TEST); glDepthFunc(GL_LEQUAL); glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE); glFrontFace(GL_CCW); glDisable(GL_BLEND);

    glBindVertexArray(vao); glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)n * sizeof(Sm64HaloVertex), mesh, GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, n);

    /* restaurar estado */
    glBindBuffer(GL_ARRAY_BUFFER, oArr); glBindVertexArray(oVao);
    glBindTexture(GL_TEXTURE_2D, oTex); glActiveTexture(oAct); glUseProgram(oProg);
    glDepthFunc(oFunc); glDepthMask(oMask); glFrontFace(oFront);
    if (!oDepth) glDisable(GL_DEPTH_TEST);
    if (!oCull) glDisable(GL_CULL_FACE);
    if (oBlend) glEnable(GL_BLEND);
}
