#version 460

layout(location = 0) in struct {
    vec4 position;
    vec3 normal;
    vec3 localNormal;
    vec3 color;
    vec2 uv;
} v_in;

layout(location = 0) out vec4 fragColor;

const vec3 lightDir = vec3(1);
const vec3 globalAmbience = vec3(0.3);

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
    vec3 color = globalAmbience * albedo + max(0, dot(N, L)) * albedo;

    fragColor = vec4(color, 1);
}