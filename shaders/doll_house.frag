#version 330 core

precision highp int;
precision highp float;
uniform sampler2D map;
uniform float opacity;
varying vec2 vUv;
vec4 white = vec4(0.5, 0.5, 0.5, 1.0);
void main() {
    vec4 colorFromTexture = texture2D( map, vUv );
    float whiteness = 1.0 - smoothstep(0.1, 0.2, opacity);
    colorFromTexture = mix(colorFromTexture, white, whiteness);
    gl_FragColor = vec4(colorFromTexture.rgb, opacity);
}