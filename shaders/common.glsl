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


float srgb2linear(float srgb) {
  if (srgb < 0.04045) {
    return srgb * (25.0 / 232.0);
  } else {
    return pow((200.0 * srgb + 11.0) * (1.0f / 211.0), 12.0 / 5.0);
  }
}

/*
float srgb2linear(float srgb) {
  return srgb <= 0.04045
       ? srgb / 12.92
       : pow((srgb + 0.055) / 1.055, 2.4);
}
*/
