// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "common.glsl"

in vec4 position;
in vec2 texcoord;
in vec3 normal;
in vec3 tangent;
in vec3 bitangent;

out vec2 x_texcoord;
out vec3 x_position;
out vec3 x_normal;
out vec3 x_tangent;
out vec3 x_bitangent;

void main() {
    gl_Position = (camera.projection_matrix * (camera.view_matrix * vec4(-position.xyz, 1.0)));
    //gl_Position = position;
    x_texcoord = texcoord;
    x_position = position.xyz;
    x_normal = normal;
    x_tangent = tangent;
    x_bitangent = bitangent;
}
