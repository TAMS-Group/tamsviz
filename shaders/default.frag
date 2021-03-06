// TAMSVIZ
// (c) 2020 Philipp Ruppel

#version 150

layout(std140) uniform material_block {
    vec4 color;
    float roughness;
    float metallic;
    int color_texture;
    int normal_texture;
    uint id;
    int flags;
} material;

struct Light {
    mat4 pose;
    vec3 color;
    int type;
    vec3 position;
    float softness;
};
layout(std140) uniform light_block {
  Light light_array[16];
  int light_count;
} lights;

uniform sampler2D color_sampler;
uniform sampler2D normal_sampler;

// uniform sampler2DArrayShadow shadow_2d_sampler;

in vec3 x_normal;
in vec4 x_position;
in vec3 x_view_position;
in vec2 x_texcoord;
in vec3 x_tangent;
in vec3 x_bitangent;
in vec4 x_color;
in vec4 x_extra;

out vec4 out_color;
out vec4 out_blend;
out uint out_id;

float srgb2linear(float srgb) {
  if (srgb < 0.04045) {
    return srgb * (25.0 / 232.0);
  } else {
    return pow((200.0 * srgb + 11.0) * (1.0f / 211.0), 12.0 / 5.0);
  }
}

void main() {
    
    const float pi = 3.14159265359;
    
    float roughness = material.roughness;
    float metallic = material.metallic;

    float roughness_2 = roughness * roughness;
    vec3 lighting = vec3(0.0, 0.0, 0.0);
    vec3 view_direction = normalize(x_view_position - x_position.xyz);
    
    if(x_extra.z > 0.5 && x_extra.z < 1.5) {
        if(length(x_texcoord) > 1.0) {
            discard;
        }
    }
    
    vec3 albedo = material.color.xyz * x_color.xyz;
    float alpha = material.color.w * x_color.w;
    
    /*
    if((material.flags & 4) != 0) {
        albedo.b = srgb2linear(x_color.r);
        albedo.g = srgb2linear(x_color.g);
        albedo.r = srgb2linear(x_color.b);
    }
    */
    
    out_id = material.id;
    
    out_blend = vec4(alpha, 0.0, 0.0, 1.0);
    if(material.color_texture != 0) {
        vec4 t = texture2D(color_sampler, x_texcoord);
        if((material.flags & 1) != 0) {
            if(t.x < 0.5) {
                discard;
            }
        } else {
            alpha *= t.w;
            albedo *= t.xyz;
        }
    }
    
    if(((material.flags & 2) != 0) || (x_extra.z > 0.1)) {
        out_color = vec4(albedo, alpha);
        return;
    }
    
    vec3 normal;
    if(material.normal_texture > 0) {
        normal = texture2D(normal_sampler, x_texcoord).xyz;
        normal.xy = normal.xy * 2.0 - 1.0;
        normal = x_tangent * normal.x + x_bitangent * -normal.y + x_normal * normal.z;
        normal = normalize(normal);
    } else {
        normal = normalize(x_normal);
    }
    
    if(dot(normal, view_direction) < 0.0) {
        normal = -normal;
    }
    
    float specular_factor = 1.0 / (0.2 + alpha);
    
    float n_dot_v = dot(normal, view_direction);
    
    vec3 fresnel_temp = mix(vec3(0.04), albedo, metallic);
    vec3 fresnel = fresnel_temp + (vec3(1.0) - fresnel_temp) * pow(1.0 - n_dot_v, 5.0);
    
    float geometry_temp = ((roughness + 1.0) * (roughness + 1.0)) / 8.0;
    float geometry = max(0.0, n_dot_v) / (max(0.0, n_dot_v) * (1.0 - geometry_temp) + geometry_temp);
    
    vec3 diffuse = (vec3(1.0) - fresnel) * (1.0 - metallic);
    
    for(int light_index = 0; light_index < lights.light_count; light_index++) {
        float light_falloff = 0.0;
        vec3 light_direction = normal;
        vec3 light_color = lights.light_array[light_index].color.xyz;
        int type = lights.light_array[light_index].type;
        switch(type) {
        case 0: { // ambient
            vec3 radiance = light_color;
            float n_dot_l = 1.0;
            float n_dot_h = 1.0;
            float ndf = 1.0;
            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_v));
            specular = min(vec3(1.0), specular);
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi + specular) * radiance * max(0.0, n_dot_l));
            continue;
        }
        case 1: { // directional
            light_falloff = 1.0;
            light_direction = transpose(mat3(lights.light_array[light_index].pose))[2];
            break;
        }
        case 2: { // point
            vec3 p = lights.light_array[light_index].position.xyz - x_position.xyz;
            light_direction = normalize(p);
            light_falloff = 1.0 / dot(p, p);
            break;
        }
        case 3: { // spot
            vec3 p = lights.light_array[light_index].position.xyz - x_position.xyz;
            light_direction = normalize(p);
            vec4 p4 = (lights.light_array[light_index].pose * x_position);
            if(p4.w > 0.0) {
                light_falloff = smoothstep(1.0, 1.0 - lights.light_array[light_index].softness, length(p4.xy / p4.w)) / dot(p, p);
            }
            break;
        }
        }
        if(light_falloff > 0.0) {
            
            vec3 half_vector = normalize(view_direction + light_direction);
            float n_dot_l = dot(normal, light_direction);
            float n_dot_h = dot(normal, half_vector);
            
            vec3 radiance = light_color * vec3(n_dot_l * light_falloff);

            float ndf_d = max(0.0, n_dot_h) * max(0.0, n_dot_h) * (roughness_2 - 1.0) + 1;
            float ndf = roughness_2 / (pi * ndf_d * ndf_d);

            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_l) * max(0.0, n_dot_v));
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi + specular) * radiance * max(0.0, n_dot_l));
            
        }
    }
    out_color = vec4(lighting, alpha);
}    
