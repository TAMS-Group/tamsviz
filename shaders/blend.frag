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
//uniform sampler2DMS id_sampler;

void main() {
    out_color = vec4(0.0, 0.0, 0.0, 1.0);
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
        //out_color.xyz += tta.xxx;
    }
    out_color.xyz *= 1.0 / float(samples);
}
