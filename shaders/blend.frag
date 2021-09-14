// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include <package://tamsviz/shaders/hdr.glsl>

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

uniform float black_level;
uniform float white_level;

void main() {
    out_color = vec4(0.0, 0.0, 0.0, 1.0);
    if(transparency) {
        for(int i = 0; i < samples; i++) {
            vec4 o = texelFetch(opaque, ivec2(gl_FragCoord.xy), i);
            vec4 t = texelFetch(transparent_head, ivec2(gl_FragCoord.xy), i);
            vec4 tta = texelFetch(transparent_tail_alpha, ivec2(gl_FragCoord.xy), i);
            if(t.w <= 0.0 && tta.x <= 0.0) {
                out_color.xyz += o.xyz;
            } else {
                vec4 ttc = texelFetch(transparent_tail_color, ivec2(gl_FragCoord.xy), i);
                vec3 ttc2 = ttc.xyz - t.xyz * t.w;
                vec3 color = mix(ttc2 / (tta.x - t.w + 1e-3), o.xyz, ttc.w / (1.0 + 1e-3 - t.w));
                color = mix(color, t.xyz, t.w);
                out_color.xyz += color;
            }
        }
    } else {
        for(int i = 0; i < samples; i++) {
            out_color.xyz += texelFetch(opaque, ivec2(gl_FragCoord.xy), i).xyz;
        }
    }
    out_color.xyz *= exposure;
    if(tone_mapping) out_color.xyz = map_color(out_color.xyz);
    out_color.xyz = (out_color.xyz - black_level) * (1.0 / (white_level - black_level));
}
