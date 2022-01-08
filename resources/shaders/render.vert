#version 460

layout(location = 0) in vec4 position;
layout(location = 1) in vec3 normal;
layout(location = 2) in vec2 uv;
layout(location = 3) in vec3 color;
layout(location = 4) in mat4 local_transform;
layout(location = 8) in vec3 scale;

layout(push_constant) uniform MVP{
    mat4 model;
    mat4 view;
    mat4 projection;
};

layout(location = 0) out struct{
    vec4 position;
    vec3 normal;
    vec3 localNormal;
    vec3 eyes;
    vec3 color;
    vec2 uv;
} v_out;


void main(){
    mat4 world_transform = model * local_transform;
    vec4 worldPosition = world_transform * position;
    vec3 worldNormal = mat3(world_transform) * normal;

    v_out.position =  position * vec4(scale, 1) * vec4(1, -1, 1, 1);
    v_out.localNormal = normal;
    v_out.normal = worldNormal;
    v_out.color = color;
    v_out.eyes = (view * vec4(0, 0, 0, 1)).xyz;
    v_out.uv = uv;
    gl_Position = projection * view * worldPosition;
}