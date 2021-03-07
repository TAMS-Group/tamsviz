// TAMSVIZ
// (c) 2020 Philipp Ruppel

#version 150

layout(std140) uniform material_block {
    vec4 color;
    float roughness;
    float metallic;
    int color_texture;
    int normal_texture;
    uint id;
    int flags;
} material;

layout(std140) uniform camera_block {
    mat4 view_matrix;
    mat4 projection_matrix;
} camera;

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

float srgb2linear(float srgb) {
  if (srgb < 0.04045) {
    return srgb * (25.0 / 232.0);
  } else {
    return pow((200.0 * srgb + 11.0) * (1.0f / 211.0), 12.0 / 5.0);
  }
}

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
    vec4 view_position = camera.view_matrix * world_position;
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
    x_view_position = -vec3(camera.view_matrix[3]) * mat3(camera.view_matrix);
    x_texcoord = texcoord;
    x_extra = extra;
    if((material.flags & 4) != 0) { // sbgr colors
        x_color.b = srgb2linear(color.r);
        x_color.g = srgb2linear(color.g);
        x_color.r = srgb2linear(color.b);
    } else {
        x_color = color;
    }
}
