#version 400

uniform sampler2D gBufferTex;
in vec2 TexCoord;

out 	vec4 	outColor;

void main(void) {
	outColor = texture(gBufferTex, TexCoord.xy);
}