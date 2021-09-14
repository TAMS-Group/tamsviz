// TAMSVIZ
// (c) 2020 Philipp Ruppel

in vec4 position;
in vec2 texcoord;

out vec2 x_texcoord;

void main() {
    gl_Position = position;
    x_texcoord = texcoord;
}
