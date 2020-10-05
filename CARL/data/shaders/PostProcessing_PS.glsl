#version 330

uniform sampler2D gBufferTex;
uniform sampler2D gTonemapLUT;

in vec2 TexCoord;

out vec4 outColor;


// Filmic tonemapping
const float _ExposureEV = exp(0.25 * 0.69314718055994530941723212145818);
const vec3 _LogLut_Params = vec3(1.0 / 1024.0, 1.0 / 32.0, 31.0);
#define COLOR_GRADING_PRECISE_LOG 0
struct ParamsLogC
{
    float cut;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
};
const ParamsLogC LogC = ParamsLogC(0.011361, 5.555556, 0.047996, 0.244161, 0.386036, 5.301883, 0.092819);
float log10(float n) 
{
	const float kLogBase10 = 0.30102999566; // 1.0 / log2(10.0)
	return log2( n ) * kLogBase10;
}
vec3 log10(vec3 n) 
{
	const float kLogBase10 = 0.30102999566; // 1.0 / log2(10.0)
	return log2( n ) * kLogBase10;
}
float LinearToLogC_Precise(float x) 
{
    float o;
    if (x > LogC.cut)
        o = LogC.c * log10(LogC.a * x + LogC.b) + LogC.d;
    else
        o = LogC.e * x + LogC.f;
    return o;
}
vec3 LinearToLogC(vec3 x)
{
#if COLOR_GRADING_PRECISE_LOG
    return vec3(
        LinearToLogC_Precise(x.x),
        LinearToLogC_Precise(x.y),
        LinearToLogC_Precise(x.z)
    );
#else
    return vec3(LogC.c) * log10(LogC.a * x + LogC.b) + vec3(LogC.d);
#endif
}
vec3 ApplyLut2d(sampler2D tex, vec3 uvw, vec3 scaleOffset) 
{
    // Strip format where `height = sqrt(width)`
    uvw.z *= scaleOffset.z;
    float shift = floor(uvw.z);
    uvw.xy = uvw.xy * scaleOffset.z * scaleOffset.xy + scaleOffset.xy * 0.5;
    uvw.x += shift * scaleOffset.y;
    uvw.xyz = mix(texture(tex, uvw.xy).rgb, texture(tex, uvw.xy + vec2(scaleOffset.y, 0)).rgb, uvw.z - shift);
    return uvw;
}


// Vignette effect
const float intensity = 0.25 * 3;
const float smoothness = 0.2 * 5;
const float roundness = 1.0;
#define saturate(x) clamp((x), 0.0, 1.0)


#define USE_COLOR_GRADING   0
#define USE_VIGNETTE_EFFECT 1


void main(void) 
{
	vec3 color = texture(gBufferTex, TexCoord.xy).rgb;
	
	// Color grading (Filmic tonemapping)
#if USE_COLOR_GRADING
	color *= _ExposureEV;
	vec3 colorLogC = saturate(LinearToLogC(color));
	color = ApplyLut2d(gTonemapLUT, colorLogC, _LogLut_Params);
#endif
	
	// Vignette effect from Unity postprocessing Stack
#if USE_VIGNETTE_EFFECT
	vec2 d = abs(TexCoord - vec2(0.5, 0.5)) * intensity;
	d = pow(saturate(d), vec2(roundness));
	float vfactor = pow(saturate(1.0 - dot(d, d)), smoothness);
	color *= vfactor;
#endif
	
	outColor = vec4(color, 1.0);
}