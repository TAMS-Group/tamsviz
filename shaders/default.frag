// TAMSVIZ
// (c) 2020 Philipp Ruppel

//#version 150
#version 400

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

uniform sampler2D color_sampler;
uniform sampler2D normal_sampler;

uniform sampler2DArrayShadow shadow_map_sampler;
uniform samplerCubeArrayShadow shadow_cube_sampler;

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
    
    if((camera.flags & uint(4)) != uint(0)) {
        out_color = vec4(0.0);
        out_blend = vec4(0.0);
        out_id = uint(0);
        return;
    }
    
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
        float shadow = 1.0;
        vec3 light_direction = normal;
        vec3 light_color = lights.light_array[light_index].color.xyz;
        int type = lights.light_array[light_index].type;
        
        vec4 p4 = vec4(0);
        if ((type & 32) != 0) {
            
            p4 = x_position;
            
            //vec4 vp4 = lights.light_array[light_index].view_matrix * p4;
            
            if ((type & 3) == 1) {
                p4.xyz += normal * (lights.light_array[light_index].shadow_bias * 2.0 / lights.light_array[light_index].projection_matrix[0][0]);
            } else {
                vec4 vp4 = lights.light_array[light_index].view_matrix * p4;
                vec4 f4 = lights.light_array[light_index].projection_matrix * vec4(0.01, 0.0, vp4.z, 1.0);
                float f = f4.w / f4.x * 0.01;
                p4.xyz += normal * (lights.light_array[light_index].shadow_bias * 2.0 * f);
            }
            
            p4 = lights.light_array[light_index].view_matrix * p4;

            p4 = lights.light_array[light_index].projection_matrix * p4;
            
            if((type & 64) != 0) {
                float smi = float(lights.light_array[light_index].shadow_index);
                vec2 smxy = p4.xy / p4.w * 0.5 + 0.5;
                float smz = p4.z / p4.w * 0.5 + 0.5;
                
                shadow = texture(shadow_map_sampler, vec4(smxy, smi, smz));
                
                /*
                if(true) {
                    shadow = 0.0;
                    for(int x = -1; x <= 1; x++) {
                        for(int y = -1; y <= 1; y++) {
                            vec2 offset = vec2(float(x), float(y)) * (1.0 / 1024.0);
                            shadow += texture(shadow_map_sampler, vec4(smxy + offset, smi, smz));
                        }
                    }
                    shadow *= (1.0 / 9.0);
                }
                */
            }
        }
        
        switch(type & 3) {
            
        /*
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
        */
        
        /*
        case 0: { // ambient
            vec3 diffuse_radiance = light_color * (normal.z * 0.5 + 0.5);
            vec3 specular_radiance = light_color * (reflect(view_direction, normal).z * -0.5 + 0.5);
            float n_dot_l = 1.0;
            float n_dot_h = 1.0;
            float ndf = 1.0;
            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_v));
            specular = min(vec3(1.0), specular);
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi * diffuse_radiance + specular * specular_radiance) * max(0.0, n_dot_l));
            continue;
        }
        */
        
        case 0: { // ambient
            vec3 light_color_2 = lights.light_array[light_index].color2.xyz;
            float hemi = lights.light_array[light_index].hemispheric;
            vec3 diffuse_radiance = mix(light_color, light_color_2, hemi * (normal.z * -0.5 + 0.5));
            vec3 specular_radiance = mix(light_color, light_color_2, hemi * (reflect(view_direction, normal).z * 0.5 + 0.5));
            float n_dot_l = 1.0;
            float n_dot_h = 1.0;
            float ndf = 1.0;
            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_v));
            specular = min(vec3(1.0), specular);
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi * diffuse_radiance + specular * specular_radiance) * max(0.0, n_dot_l));
            continue;
        }
        
        case 1: { // directional
            light_falloff = 1.0;
            light_direction = transpose(mat3(lights.light_array[light_index].view_matrix))[2];
            break;
        }
        
        case 2: { // point
            
            vec3 p = lights.light_array[light_index].position.xyz - x_position.xyz;
            light_direction = normalize(p);
            light_falloff = 1.0 / dot(p, p);
            
            if((type & 128) != 0) {
                
                //p -= normal * (lights.light_array[light_index].shadow_bias * length(p) * 2.0);
                p -= normal * (lights.light_array[light_index].shadow_bias * max(abs(p.x), max(abs(p.y), abs(p.z))) * 2.0);
                //p -= normal * (lights.light_array[light_index].shadow_bias);
                
                float ref = max(abs(p.x), max(abs(p.y), abs(p.z)));
                
                float smi = float(lights.light_array[light_index].shadow_index);
                ref = -ref;
                
                float far = 20.0;
                float near = 0.1;
                
                mat4 proj = mat4(0);
                proj[0][0] = 0.0;
                proj[1][1] = 0.0;
                proj[2][2] = -far / (far - near);
                proj[2][3] = -far * near / (far - near);
                proj[3][2] = -1.0;
                
                vec4 proj4 = vec4(0.0, 0.0, ref, 1.0) * proj;
                ref = proj4.z / proj4.w;
                ref = ref * 0.5 + 0.5;

                shadow = texture(shadow_cube_sampler, vec4(-p, smi), ref);
            }
            
            break;
        }
        
        case 3: { // spot
            vec3 p = lights.light_array[light_index].position.xyz - x_position.xyz;
            light_direction = normalize(p);
            if(p4.w > 0.0) {
                light_falloff = smoothstep(1.0, 1.0 - lights.light_array[light_index].softness, length(p4.xy / p4.w)) / dot(p, p);
            }
            break;
        }
        
        }
        
        light_falloff *= shadow;
        
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
