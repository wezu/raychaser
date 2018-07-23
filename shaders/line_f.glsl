//GLSL
#version 140
uniform vec4 p3d_Color;

in flat float vert_order;

void main()
    {
    vec4 color=p3d_Color;
    if (vert_order < 0.5)
        color.rgb*=0.7;
    gl_FragData[0]=color;
    }
