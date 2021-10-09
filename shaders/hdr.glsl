// HDL Tone Mapping, based on 
// https://en.wikipedia.org/wiki/Academy_Color_Encoding_System
// https://docs.nvidia.com/gameworks/content/devices/shield-hdr-dev-guide/hdr-dev-guide-nvidia-shield.htm
// https://github.com/TheRealMJP/BakingLab/blob/master/BakingLab/ACES.hlsl

const mat3 tone_input = mat3(
    vec3(0.59719, 0.35458, 0.04823),
    vec3(0.07600, 0.90834, 0.01566),
    vec3(0.02840, 0.13383, 0.83777)
);

const mat3 tone_input_inverse = mat3(
    vec3( 1.76474097, -0.67577768, -0.08896329),
    vec3(-0.14702785,  1.16025151, -0.01322366),
    vec3(-0.03633683, -0.16243644,  1.19877327)  
);

const mat3 tone_output = mat3(
    vec3(1.60475, -0.53108, -0.07367),
    vec3(-0.10208, 1.10813, -0.00605),
    vec3(-0.00327, -0.07276, 1.07602)
);

const mat3 tone_output_inverse = mat3(
    vec3(0.64303825, 0.31118675, 0.04577546),
    vec3(0.05926869, 0.93143649, 0.00929492),
    vec3(0.0059619 , 0.06392902, 0.93011838)
);

vec3 tone_transform(vec3 x) {
    return (x * (x + 0.0245786) - 0.000090537) / (x * (0.983729 * x + 0.4329510) + 0.238081);
}

vec3 tone_transform_inverse(vec3 y) {
    return (-3.21458e-8 * sqrt(-187345541948750.0 * y * y + 232671271403227.0 * y + 241563894490.0) - 0.220056 * y + 0.0124926) / (y - 1.01654); 
}

vec3 map_color_inverse(vec3 color) {
    
    //color = clamp(color, vec3(0.0), vec3(1.0));
    
    color = color * tone_output_inverse;
    
    color = tone_transform_inverse(color);
    
    color = color * tone_input_inverse;
    
    return color;
}

vec3 map_color(vec3 color) {
    
    color = color * tone_input;
    
    color = tone_transform(color);
    
    //color = tone_transform_inverse(color);
    //color = tone_transform(color);
    
    color = color * tone_output;
    
    return color;
}
