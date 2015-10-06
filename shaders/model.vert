#version 330 core

precision highp int;
precision highp float;
uniform mat4 modelMatrix;
uniform mat4 modelViewMatrix;
uniform mat4 projectionMatrix;
attribute vec3 position;
attribute vec2 uv;
uniform vec3 pano0Position;
uniform mat4 pano0Matrix;
uniform vec3 pano1Position;
uniform mat4 pano1Matrix;
varying vec2 vUv;
varying vec3 vWorldPosition0;
varying vec3 vWorldPosition1;

void main() {
    vUv = uv;
    vec4 worldPosition = modelMatrix * vec4(position, 1.0);
    vec3 positionLocalToPanoCenter0 = worldPosition.xyz - pano0Position;
    vWorldPosition0 = (vec4(positionLocalToPanoCenter0, 1.0) * pano0Matrix).xyz;
    vWorldPosition0.x *= -1.0;
    vec3 positionLocalToPanoCenter1 = worldPosition.xyz - pano1Position;
    vWorldPosition1 = (vec4(positionLocalToPanoCenter1, 1.0) * pano1Matrix).xyz;
    vWorldPosition1.x *= -1.0;
    gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );
}