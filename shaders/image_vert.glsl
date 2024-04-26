// TAMSVIZ
// (c) 2024 Philipp Ruppel

varying vec2 x_texcoord;

void main() {
    // gl_Position = gl_Vertex;
    gl_Position = ftransform();
    // x_texcoord = texcoord;
    x_texcoord = gl_MultiTexCoord0;
}
