//GLSL
#version 130
uniform sampler2D depth_tex;
uniform vec3 color;

uniform float thickness;

in vec2 uv;

void main()
    {

    float f=500.0;
    float n = 0.01;
    float z = (2.0 * n) / (f + n - texture( depth_tex, uv ).x * (f - n));
    float separation = thickness*0.001*(1.0-z);

    float color0=(2.0 * n) / (f + n - texture( depth_tex, uv+vec2(separation, 0.0)).x* (f - n));
    float color1=(2.0 * n) / (f + n - texture( depth_tex, uv-vec2(separation, 0.0)).x* (f - n));
    float color2=(2.0 * n) / (f + n - texture( depth_tex, uv+vec2(0.0, separation)).x* (f - n));
    float color3=(2.0 * n) / (f + n - texture( depth_tex, uv-vec2(0.0, separation)).x* (f - n));

    float mx = max(color0,max(color1,max(color2,color3)));
    float mn = min(color0,min(color1,min(color2,color3)));

    float line=step(0.002, (mx-mn));

    gl_FragData[0]=vec4(color.rgb, line);
    }
