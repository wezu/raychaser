//GLSL
#version 140
uniform mat4 p3d_ModelViewProjectionMatrix;
in vec4 p3d_Vertex;
out flat float vert_order;

void main()
    {
    vert_order= mod(gl_VertexID, 2.0);
    gl_Position = p3d_ModelViewProjectionMatrix * p3d_Vertex;
    }
