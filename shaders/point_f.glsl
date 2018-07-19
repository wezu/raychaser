//GLSL
#version 140
uniform vec4 p3d_Color;


void main()
    {
    gl_FragData[0]=p3d_Color;
    }
