//GLSL
#version 140
uniform sampler2D litex;
uniform vec4 p3d_Color;
uniform float gloss;

in vec3 N;
in vec3 V;


void main()
    {
    vec4 color=p3d_Color;
    vec3 normal=normalize(N);   
    //vec3 flat_normal = normalize( cross(dFdx(V), dFdy(V)) );
    
    vec4 lit=texture(litex, vec2(normal.xyz * vec3(0.495) + vec3(0.5)));

    float mate=max(0.0, ((1.0-gloss)-0.5)*2.0);//mate
    float glossy=max(0.0, (gloss-0.5)*2.0);//gloss
    float mid =1.0 -mate -glossy;//mid

    color.rgb=p3d_Color.rgb*mate*pow(lit.r, 1.66);
    color.rgb+=p3d_Color.rgb*glossy*pow(lit.b, 1.66);
    color.rgb+=p3d_Color.rgb*mid*pow(lit.g, 1.66);

    gl_FragData[0]=color;
    }
