// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

// #include <package://tamsviz/shaders/common.glsl>
#include "common.glsl"

in vec4 position;
in vec3 normal;
in vec2 texcoord;
in vec3 tangent;
in vec3 bitangent;
in vec4 pose_x;
in vec4 pose_y;
in vec4 pose_z;
in vec4 extra;
in vec4 color;

out vec3 x_normal;
out vec4 x_position;
out vec3 x_view_position;
out vec2 x_texcoord;
out vec3 x_tangent;
out vec3 x_bitangent;
out vec4 x_color;
out vec4 x_extra;

void main() {
    mat4 world_matrix = transpose(mat4(pose_x, pose_y, pose_z, vec4(0.0, 0.0, 0.0, 1.0)));
    mat3 normal_matrix = mat3(world_matrix);
    normal_matrix[0] *= 1.0 / (dot(normal_matrix[0], normal_matrix[0]));
    normal_matrix[1] *= 1.0 / (dot(normal_matrix[1], normal_matrix[1]));
    normal_matrix[2] *= 1.0 / (dot(normal_matrix[2], normal_matrix[2]));
    x_normal = normal_matrix * normal;
    x_tangent = normal_matrix * tangent;
    x_bitangent = normal_matrix * bitangent;
    vec4 world_position = world_matrix * position;
    x_view_position = -vec3(camera.view_matrix[3]) * mat3(camera.view_matrix);
    if((material.flags & 8) != 0) {
        world_position.xyz = world_position.xyz + x_view_position;
    }
    vec4 view_position = camera.view_matrix * world_position;
    /*
    if((camera.flags & uint(4)) != uint(0)) {
        gl_Position = (camera.projection_matrix * view_position);
        return;
    }
    */
    if(extra.z == 1.0) { // point
        view_position.xy += texcoord * extra.x;
    }
    if(extra.z == 2.0) { // line
        view_position.xyz -= normalize(cross(mat3(camera.view_matrix) * x_tangent, view_position.xyz)) * extra.x;
    }
    if(extra.z == 3.0) { // text
        view_position.xy += extra.xy;
    }
    gl_Position = (camera.projection_matrix * view_position);
    x_position = world_position;
    x_texcoord = texcoord;
    x_extra = extra;
    if((material.flags & 4) != 0) { // sbgr colors
        x_color.b = srgb2linear(color.r);
        x_color.g = srgb2linear(color.g);
        x_color.r = srgb2linear(color.b);
    } else {
        x_color = color;
    }

    /*if(true) { // todo: remove again
      x_color.r = srgb2linear(color.r);
      x_color.g = srgb2linear(color.g);
      x_color.b = srgb2linear(color.b);
    }*/
}
