#version 450

layout (binding = 0) uniform sampler2D texSampler;

layout (location = 0) in vec2 fragTexCoord;

layout (location = 0) out vec4 outFragColor;

void main() 
{
    outFragColor = texture(texSampler, fragTexCoord).zyxw; 
}
