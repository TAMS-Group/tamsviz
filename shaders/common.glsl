// TAMSVIZ
// (c) 2020 Philipp Ruppel

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
};

layout(std140) uniform light_block {
  Light light_array[16];
  int light_count;
} lights;

float srgb2linear(float srgb) {
  if (srgb < 0.04045) {
    return srgb * (25.0 / 232.0);
  } else {
    return pow((200.0 * srgb + 11.0) * (1.0f / 211.0), 12.0 / 5.0);
  }
}
