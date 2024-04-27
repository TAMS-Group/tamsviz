// TAMSVIZ
// (c) 2024 Philipp Ruppel

#include "srgb.glsl"
#include "hdr.glsl"

varying vec2 x_texcoord;
uniform sampler2D texture0;

uniform float brightness;
uniform float saturation;
uniform bool tonemapping;
uniform int channels;

void main() {

    vec4 c = texture2D(texture0, x_texcoord);

    if (channels == 1)
        c.xyz = vec3(c.x);

    for(int i = 0; i < 3; i++)
        c[i] = srgb2linear(c[i]);

    c.xyz *= brightness;
    c.xyz = mix(vec3(dot(c.xyz, vec3(1.0 / 3.0))), c, saturation);

    c = max(c, vec4(0.0));

    if (tonemapping)
        c.xyz = map_color(c.xyz);

    for(int i = 0; i < 3; i++)
        c[i] = linear2srgb(c[i]);

    gl_FragColor = c;

}
 