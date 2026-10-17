#version 450
layout(push_constant) uniform PC { mat4 vp; } pc;
layout(location = 0) in vec3 pos;
layout(location = 1) in vec4 col;
layout(location = 0) out vec4 vcol;
void main() {
    gl_Position = pc.vp * vec4(pos, 1.0);
    vcol = col;
}
