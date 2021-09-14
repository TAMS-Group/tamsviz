// HDL Tone Mapping, based on 
// https://en.wikipedia.org/wiki/Academy_Color_Encoding_System
// https://docs.nvidia.com/gameworks/content/devices/shield-hdr-dev-guide/hdr-dev-guide-nvidia-shield.htm
// https://github.com/TheRealMJP/BakingLab/blob/master/BakingLab/ACES.hlsl

const mat3 tone_input = mat3(
    vec3(0.59719, 0.35458, 0.04823),
    vec3(0.07600, 0.90834, 0.01566),
    vec3(0.02840, 0.13383, 0.83777)
);

const mat3 tone_output = mat3(
    vec3(1.60475, -0.53108, -0.07367),
    vec3(-0.10208, 1.10813, -0.00605),
    vec3(-0.00327, -0.07276, 1.07602)
);

vec3 tone_transform(vec3 input) {
    vec3 n = input * (input + 0.0245786) - 0.000090537;
    vec3 d = input * (0.983729 * input + 0.4329510) + 0.238081;
    return n / d;
}

vec3 map_color(vec3 color) {
    color = color * tone_input;
    color = tone_transform(color);
    color = color * tone_output;
    return color;
}
