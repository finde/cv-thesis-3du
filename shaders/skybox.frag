#version 330 core
in vec3 TexCoords;
out vec4 color;

uniform samplerCube skybox1;
uniform samplerCube skybox2;
uniform float progress;
uniform vec3 pos;

void main()
{
    // pos = TexCoords;

    // move toward skybox2
    vec4 color_sky1 = texture(skybox1, TexCoords);

    // stay
    vec4 color_sky2 = texture(skybox2, TexCoords);

    // transition from skybox1 to skybox2
    color = mix(color_sky1, color_sky2, progress);
}