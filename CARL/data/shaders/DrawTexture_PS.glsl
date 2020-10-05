 #version 330

uniform sampler2D gTex;

in vec2 TexCoord;

out vec4 outColor;

void main(void) 
{
	vec3 color = texture(gTex, TexCoord.xy).rgb;
	outColor = vec4(color, 1.0);
}
