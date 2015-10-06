#version 330 core

precision highp int;
precision highp float;
uniform sampler2D map;
uniform float modelAlpha;
uniform float opacity;
uniform float progress;
uniform lowp samplerCube pano0Map;
uniform lowp samplerCube pano1Map;
varying vec2 vUv;
varying vec3 vWorldPosition0;
varying vec3 vWorldPosition1;

void main() {
    vec4 colorFromPano0 = textureCube( pano0Map, vWorldPosition0.xyz );
    vec4 colorFromPano1 = textureCube( pano1Map, vWorldPosition1.xyz );
    vec4 colorFromPanos = mix(colorFromPano0, colorFromPano1, progress);
    vec4 colorFromTexture = texture2D( map, vUv );
    colorFromPanos = mix(colorFromPanos, colorFromTexture, modelAlpha);
    float whiteness = 1.0 - smoothstep(0.1, 0.2, opacity);
    colorFromPanos = mix(colorFromPanos, vec4(0.5, 0.5, 0.5, 1.0), whiteness);
    gl_FragColor = vec4(colorFromPanos.rgb, opacity);
}
