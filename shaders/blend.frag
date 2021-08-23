// TAMSVIZ
// (c) 2020 Philipp Ruppel

#version 150

in vec2 x_texcoord;

out vec4 out_color;

uniform int samples;
uniform sampler2DMS opaque;
uniform sampler2DMS transparent_head;
uniform sampler2DMS transparent_tail_color;
uniform sampler2DMS transparent_tail_alpha;

uniform bool transparency;

uniform float exposure;
uniform bool tone_mapping;

vec3 map_color(vec3 color);

void main() {
    out_color = vec4(0.0, 0.0, 0.0, 1.0);
    if(transparency) {
        for(int i = 0; i < samples; i++) {
            vec4 o = texelFetch(opaque, ivec2(gl_FragCoord.xy), i);
            vec4 t = texelFetch(transparent_head, ivec2(gl_FragCoord.xy), i);
            vec4 tta = texelFetch(transparent_tail_alpha, ivec2(gl_FragCoord.xy), i);
            //vec3 c = o.xyz;
            if(t.w <= 0.0 && tta.x <= 0.0) {
                out_color.xyz += o.xyz;
            } else {
                vec4 ttc = texelFetch(transparent_tail_color, ivec2(gl_FragCoord.xy), i);
                vec3 ttc2 = ttc.xyz - t.xyz * t.w;
                vec3 color = mix(ttc2 / (tta.x - t.w + 1e-3), o.xyz, ttc.w / (1.0 + 1e-3 - t.w));
                color = mix(color, t.xyz, t.w);
                out_color.xyz += color;
            }
            //if(tone_mapping) c = map_color(c * exposure);
            //out_color.xyz += c;
        }
    } else {
        for(int i = 0; i < samples; i++) {
            vec3 c = texelFetch(opaque, ivec2(gl_FragCoord.xy), i).xyz;
            //if(tone_mapping) c = map_color(c * exposure);
            out_color.xyz += c;
        }
    }
    out_color.xyz *= exposure;
    if(tone_mapping) out_color.xyz = map_color(out_color.xyz);
}

// tone mapping, based on 
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
