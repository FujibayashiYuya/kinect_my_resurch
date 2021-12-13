const float k1 = 0.429043;
const float k2 = 0.511664;
const float k3 = 0.743125;
const float k4 = 0.886227;
const float k5 = 0.247708;

uniform float shy[9];
uniform vec3 sh[9];

void main()
{
    gl_Position = ftransform();
}