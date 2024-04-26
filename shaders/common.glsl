// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

layout(std140) uniform material_block {
    vec4 color;
    float roughness;
    float metallic;
    int color_texture;
    int normal_texture;
    uint id;
    int flags;
    float brightness;
} material;

layout(std140) uniform camera_block {
    mat4 view_matrix;
    mat4 projection_matrix;
    uint flags;
} camera;

struct Light {
    mat4 view_matrix;
    mat4 projection_matrix;
    vec3 color;
    int type;
    vec3 color2;
    float hemispheric;
    vec3 position;
    float softness;
    float shadow_bias;
    int shadow_index;
    float reserved0;
    float reserved1;
};

layout(std140) uniform light_block {
  Light light_array[16];
  int light_count;
} lights;
