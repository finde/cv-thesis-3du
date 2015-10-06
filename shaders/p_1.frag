precision highp float;
precision highp int;

uniform mat4 viewMatrix;
uniform vec3 cameraPosition;
varying vec4 worldPosition;
uniform float radius;

void main() {
    vec4 topColor = vec4(0.094, 0.102, 0.11, 1.0);
    vec4 bottomColor = vec4(0.2, 0.216, 0.235, 1.0);
    float normalizedHeight = (worldPosition.y + radius) / (radius * 2.0);
    float ratio = smoothstep(0.0, 0.5, normalizedHeight);
    gl_FragColor = mix(bottomColor, topColor, ratio);
}
