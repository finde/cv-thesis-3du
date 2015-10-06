#version 330 core
in vec3 TexCoords;

out vec4 color;

uniform samplerCube skybox1;
uniform samplerCube skybox2;
uniform float progress;
uniform vec3 pos;
uniform vec3 vWorldPosition1;
uniform vec3 vWorldPosition2;

void main()
{
    // pos = TexCoords;

    // move toward skybox2
    vec4 color_sky1 = texture(skybox1, TexCoords + vWorldPosition1);

    // stay
    vec4 color_sky2 = texture(skybox2, TexCoords + vWorldPosition2);

    // transition from skybox1 to skybox2
    color = mix(color_sky1, color_sky2, progress);
}