#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec2 inTexCoord;

layout(location = 0) out vec2 fragTexCoord;

out gl_PerVertex {
	vec4 gl_Position;   
};

layout(push_constant) uniform PushConsts {
	mat4 mvp;
} pushConsts;

void main() 
{
	fragTexCoord = inTexCoord;
	gl_Position = pushConsts.mvp * vec4(inPos.xyz, 1.0);
}
