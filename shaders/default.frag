// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

// material.flags
// 1: alpha testing
// 2: unlit
// 4: sbgr vertex colors
// 8: viewspace
// 32: glow
// 64: envmap?

// #include <package://tamsviz/shaders/common.glsl>
#include "common.glsl"
#include "hdr.glsl"

uniform sampler2D color_sampler;
uniform sampler2D normal_sampler;

uniform sampler2DArrayShadow shadow_map_sampler;
uniform samplerCubeArrayShadow shadow_cube_sampler;

uniform samplerCube environment_sampler;

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

void main() {

    /*{
      out_color = x_color;
      return;
    }*/

    bool glow = ((material.flags & 32) != 0);

    const float pi = 3.14159265359;

    if(x_extra.z > 0.5 && x_extra.z < 1.5) {
        if(length(x_texcoord) > 1.0) {
            discard;
        }
    }

    vec3 albedo = material.color.xyz;
    float alpha = material.color.w;

    if(!glow) {
        albedo *= x_color.xyz;
        alpha *= x_color.w;
    }

    out_id = material.id;

    bool unlit = (((material.flags & 2) != 0) || (x_extra.z > 0.1) || (material.metallic < 0.0));

    out_blend = vec4(alpha, 0.0, 0.0, 1.0);
    if(material.color_texture != 0) {

        vec4 t = texture2D(color_sampler, x_texcoord);

        // vec4 t = textureGrad(color_sampler, x_texcoord, 
        //     vec2(dFdx(x_texcoord.x),dFdy(x_texcoord.x))*30, 
        //     vec2(dFdx(x_texcoord.y),dFdy(x_texcoord.y))*30
        //     );

        if((material.flags & 1) != 0) {
            if(t.x < 0.5) {
                discard;
            }
        } else {
            alpha *= t.w;
            albedo *= t.xyz;
            alpha = max(0.0, min(alpha, 1.0));
            albedo = max(vec3(0.0), min(albedo, vec3(1.0)));
        }
    }

    if((camera.flags & 4) != 0) { // rendering shadow map
        out_color = vec4(0.0);
        return;
    }

    if(unlit) {
        vec3 c = albedo;
        c = map_color_inverse(c);
        c *= material.brightness;
        out_color = vec4(c, alpha);
        return;
    }

    albedo *= material.brightness;

    float roughness = material.roughness;
    float metallic = material.metallic;

    float roughness_2 = roughness * roughness;
    vec3 lighting = vec3(0.0, 0.0, 0.0);
    vec3 view_direction = normalize(x_view_position - x_position.xyz);

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

    //float specular_factor = 1.0 / (0.2 + alpha);
    float specular_factor = 1.0;

    float n_dot_v = dot(normal, view_direction);

    vec3 fresnel_temp = mix(vec3(0.04), albedo, metallic);
    vec3 fresnel = fresnel_temp + (vec3(1.0) - fresnel_temp) * pow(1.0 - n_dot_v, 5.0);

    float geometry_temp = ((roughness_2 + 1.0) * (roughness_2 + 1.0)) / 8.0;
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

            int shadow_sample_range = 3;

            // float shadow_bias = lights.light_array[light_index].shadow_bias * 2.0;
            float shadow_bias = lights.light_array[light_index].shadow_bias * 2.0 * (1.0 + shadow_sample_range);

            if ((type & 3) == 1) {
                p4.xyz += normal * (shadow_bias / lights.light_array[light_index].projection_matrix[0][0]);
            } else {
                vec4 vp4 = lights.light_array[light_index].view_matrix * p4;
                vec4 f4 = lights.light_array[light_index].projection_matrix * vec4(0.01, 0.0, vp4.z, 1.0);
                float f = f4.w / f4.x * 0.01;
                p4.xyz += normal * (shadow_bias * f);
            }

            p4 = lights.light_array[light_index].view_matrix * p4;

            p4 = lights.light_array[light_index].projection_matrix * p4;

            if((type & 64) != 0) {
                float smi = float(lights.light_array[light_index].shadow_index);
                
                // vec2 smxy = p4.xy / p4.w * 0.5 + 0.5;
                // float smz = p4.z / p4.w * 0.5 + 0.5;
                // shadow = texture(shadow_map_sampler, vec4(smxy, smi, smz));

                vec2 smstep = 1.0 / vec2(textureSize(shadow_map_sampler, 0));

                shadow = 0.0;
                for(int ox = -shadow_sample_range; ox <= shadow_sample_range; ox++) {
                    for(int oy = -shadow_sample_range; oy <= shadow_sample_range; oy++) {
                        vec2 off = vec2(float(ox), float(oy)) * smstep;
                        vec2 smxy = p4.xy / p4.w * 0.5 + 0.5 + off * 1;
                        float smz = p4.z / p4.w * 0.5 + 0.5;
                        shadow += texture(shadow_map_sampler, vec4(smxy, smi, smz));
                    }
                }
                shadow *= 1.0 / ((shadow_sample_range * 2 + 1) * (shadow_sample_range * 2 + 1));
            }
        }

        switch(type & 7) {

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


        case 4: {

            vec3 dir = reflect(view_direction, normal);
            //lighting += texture(environment_sampler, dir).xyz;
            //lighting += vec3(1.0);

            vec3 diffuse_radiance = max(vec3(0.0), textureLod(environment_sampler, dir, 32.0).xyz * light_color);

            //float level = pow(roughness, 1.0 / 1.7) * (10.0 - 1.0);
            float level = sqrt(roughness) * 9.0;
            vec3 specular_radiance = max(vec3(0.0), textureLod(environment_sampler, dir, level).xyz * light_color);

            // vec3 dx = dFdx(dir);
            // vec3 dy = dFdy(dir);
            // dx += normalize(dx) * (roughness * 1.0);
            // dy += normalize(dy) * (roughness * 1.0);
            // vec3 specular_radiance = max(vec3(0.0), textureGrad(environment_sampler, dir, dx, dy).xyz * light_color);

            float n_dot_l = 1.0;
            float n_dot_h = 1.0;
            float ndf = 1.0;
            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_v));
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi * diffuse_radiance + specular * specular_radiance) * max(0.0, n_dot_l));


            //lighting += specular_radiance;

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

                p -= normal * (lights.light_array[light_index].shadow_bias * max(abs(p.x), max(abs(p.y), abs(p.z))) * 2.0);

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

            float ndf_d = max(0.0, n_dot_h) * max(0.0, n_dot_h) * (roughness_2 - 1.0) + 1.0;
            float ndf = roughness_2 / (pi * ndf_d * ndf_d);

            vec3 specular = (vec3(ndf * geometry) * fresnel) / max(0.001, 4.0 * max(0.0, n_dot_l) * max(0.0, n_dot_v));
            specular *= specular_factor;
            lighting += max(vec3(0.0), (diffuse * albedo / pi + specular) * radiance * max(0.0, n_dot_l));

        }
    }
    out_color = vec4(lighting, alpha);

    if (glow) {
        vec3 v = x_color.rgb;
        v = max(vec3(0.0), v);
        v *= v;
        out_color.rgb += v;
    }
}
