// TAMSVIZ
// (c) 2024 Philipp Ruppel

varying vec2 x_texcoord;

void main() {
    // gl_Position = gl_Vertex;
    gl_Position = ftransform();
    // x_texcoord = texcoord;
    x_texcoord.x = gl_MultiTexCoord0.x;
    x_texcoord.y = 1.0 - gl_MultiTexCoord0.y;
}
