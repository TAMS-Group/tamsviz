// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "common.glsl"
#include "hdr.glsl"

uniform sampler2D color_sampler;

uniform int level;
uniform int levels;

in vec2 x_texcoord;
in vec3 x_position;
in vec3 x_normal;
in vec3 x_tangent;
in vec3 x_bitangent;

out vec4 out_color;

float atan2(float y, float x) {
    const float pi = 3.14159265359;
    return mix(pi / 2.0 - atan(x, y), atan(y, x), abs(x) > abs(y) ? 1.0 : 0.0);
}

vec3 sampleEnvironment(vec3 dir, float smoothing) {
    const float pi = 3.14159265359;
    dir = normalize(dir);
    float y = asin(dir.z) / pi + 0.5;
    float x = atan2(dir.x, dir.y) / (2.0 * pi);
    vec3 color = textureGrad(
        color_sampler,
        vec2(x - 0.25, 1.0 - y), 
        vec2(smoothing /*/ length(dir.xy)*/, 0.0), 
        vec2(0.0, smoothing)
        ).xyz;
    color = map_color_inverse(color);
    return color;
}

float random_number(uint i) {
    //const float pi = 3.14159265359;
    //return fract(float(i) * pi + pi);
    
    /*i *= 3128473621;
    i >>= 24;
    i *= 1523241233;
    i >>= 24;
    i *= 2184971823;
    i >>= 24;*/
    
    i ^= i * 0x6c50b47cu;
    i ^= i * 0xb82f1e52u;
    i ^= i * 0xc7afe638u;
    i ^= i * 0x8d22f6e6u;
    i >>= 24;
    
    return (float(i & 0xff) * (1.0 / 255.0));
}

void main() {
    
    out_color.w = 1.0;
    
    // out_color.xyz = sampleEnvironment(normalize(-x_position), 0.0);
    // return;
    
    
    const float pi = 3.14159265359;
    
    //float roughness = (float(level) / float(levels - 1));
    float roughness = (float(level) / 9.0);
    roughness = roughness * roughness;
    roughness = roughness * roughness;
    
    //roughness *= 0.0;
    
    float smoothing = roughness * 0.1;
    
    out_color.xyz = vec3(0.0);
    
    const int n = 256;
    
    vec3 vz = normalize(x_position);
    vec3 vx = normalize(cross(vec3(1.0, 2.0, 3.0), vz));
    vec3 vy = normalize(cross(vx, vz));
    
    uint rng0 = (uint(gl_FragCoord.x) * n + (uint(gl_FragCoord.y) << 10)) * n * 4;
    
    for(int i = 0; i < n; i++) {
        
        rng0 += 4;
        
        uint rng = rng0;
        
        //rng *= 129737121;
        //rng = fract(float(rng) * 1.61803398874989484820459);
        
        float rx = random_number(rng + 0u);
        float ry = random_number(rng + 1u);
        float rz = random_number(rng + 2u);
        float rw = random_number(rng + 3u);
        
        //rng = fract(sin(rng) * 100.0);
        
        /*float rx = fract(noise1(float(rng) + 0.142) * 125123.0);
        float ry = fract(noise1(float(rng) + 1.634) * 654634.0);
        float rz = fract(noise1(float(rng) + 2.235) * 763342.0);
        float rw = fract(noise1(float(rng) + 3.765) * 125243.0);*/
        
        // float rx = float((rng >> 0) & 0xff) * (1.0 / 255.0);
        // float ry = float((rng >> 8) & 0xff) * (1.0 / 255.0);
        // float rz = float((rng >> 16) & 0xff) * (1.0 / 255.0);
        // float rw = float((rng >> 24) & 0xff) * (1.0 / 255.0);
        
        float gx = sqrt(-2.0 * log(rx)) * cos(2.0 * pi * ry) * roughness;
        float gy = sqrt(-2.0 * log(ry)) * sin(2.0 * pi * rx) * roughness; 
        float gz = sqrt(-2.0 * log(rz)) * cos(2.0 * pi * rw) * roughness;
        
        //vec3 norm = normalize(vz + vx * gx + vy * gy);
        
        vec3 norm = normalize(vz + vec3(gx, gy, gz));
        
        //vec3 dir = reflect(vz, norm);
        vec3 dir = -norm;
        
        out_color.xyz += sampleEnvironment(dir, smoothing);
        
    }
    
    out_color.xyz *= (1.0 / float(n));

    
    // out_color.xyz = x_position * 1.0 + 0.5;
    
}
