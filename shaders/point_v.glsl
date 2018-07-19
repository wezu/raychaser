//GLSL
#version 140

uniform mat4 p3d_ModelViewProjectionMatrix;
uniform vec2 screen_size;
uniform vec3 camera_pos;

in vec4 p3d_Vertex;


void main()
    {
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
    gl_PointSize = (screen_size.y/distance(p3d_Vertex.xyz, camera_pos))*0.09;
    }
