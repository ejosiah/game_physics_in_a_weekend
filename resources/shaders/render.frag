#version 460

layout(set = 1, binding = 0) uniform sampler2D shadowMap;

layout(location = 0) in struct {
    vec4 lightSpacePos;
    vec4 position;
    vec3 normal;
    vec3 localNormal;
    vec3 eyes;
    vec3 color;
    vec2 uv;
} v_in;

layout(location = 0) out vec4 fragColor;

const vec3 lightDir = vec3(1);
const vec3 globalAmbience = vec3(0.3);

float shadowCalculation(vec4 lightSpacePos){
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords.xy = projCoords.xy * 0.5 + 0.5;
    if(projCoords.z > 1.0){
        return 0.0;
    }
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;
    float shadow = currentDepth > closestDepth ? 1.0 : 0.0;
    return shadow;
}
float pcfFilteredShadow(vec4 lightSpacePos){
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords.xy = projCoords.xy * 0.5 + 0.5;
    if(projCoords.z > 1.0){
        return 0.0;
    }
    float shadow = 0.0f;
    float currentDepth = projCoords.z;
    vec2 texelSize = 1.0/textureSize(shadowMap, 0);
    for(int x = -1; x <= 1; x++){
        for(int y = -1; y <= 1; y++){
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth > pcfDepth ? 1.0 : 0.0;
        }
    }
    return shadow/9.0;
}

vec3 GetColorFromPositionAndNormal( in vec3 worldPosition, in vec3 normal ) {
    const float pi = 3.141519;

    vec3 scaledPos = worldPosition.xyz * pi * 2.0;
    vec3 scaledPos2 = worldPosition.xyz * pi * 2.0 / 10.0 + vec3( pi / 4.0 );
    float s = cos( scaledPos2.x ) * cos( scaledPos2.y ) * cos( scaledPos2.z );  // [-1,1] range
    float t = cos( scaledPos.x ) * cos( scaledPos.y ) * cos( scaledPos.z );     // [-1,1] range

    vec3 colorMultiplier = vec3( 0.5, 0.5, 1 );
    if ( abs( normal.x ) > abs( normal.y ) && abs( normal.x ) > abs( normal.z ) ) {
        colorMultiplier = vec3( 1, 0.5, 0.5 );
    } else if ( abs( normal.y ) > abs( normal.x ) && abs( normal.y ) > abs( normal.z ) ) {
        colorMultiplier = vec3( 0.5, 1, 0.5 );
    }


    t = ceil( t * 0.9 );
    s = ( ceil( s * 0.9 ) + 3.0 ) * 0.25;
    vec3 colorB = vec3( 0.85, 0.85, 0.85 );
    vec3 colorA = vec3( 1, 1, 1 );
    vec3 finalColor = mix( colorA, colorB, t ) * s;

    return colorMultiplier * finalColor;
}


void main(){
    vec3 N = normalize(v_in.normal);
    vec3 L = normalize(lightDir);
    vec3 E = normalize(v_in.eyes);
    vec3 H = normalize(E + L);

    vec3 albedo = vec3(0);
    float dx = 0.25;
    float dy = 0.25;
    vec4 modelPos = v_in.position;
    for(float y = 0.0; y < 1.0; y+= dy){
        for(float x = 0.0; x < 1.0; x += dx){
            vec4 samplePos = modelPos + dFdx(modelPos) * x + dFdy(modelPos) * y;
            albedo += GetColorFromPositionAndNormal( samplePos.xzy, v_in.localNormal.xzy) * dx * dy;
        }
    }
    float shadow = pcfFilteredShadow(v_in.lightSpacePos);
    float diffuse = max(0, dot(N, L));
    float specular = max(0, pow(dot(N, H), 1000));
    vec3 color = albedo * (globalAmbience +  (1 - shadow) * diffuse);

    fragColor = vec4(color, 1);
}