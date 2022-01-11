const float k1 = 0.429043;
const float k2 = 0.511664;
const float k3 = 0.743125;
const float k4 = 0.886227;
const float k5 = 0.247708;

uniform vec2 irradiance;
uniform float shy[9];
uniform float clm[9];

void main(void)
{
	vec3 n;
	float x;
	float y;
	float ir;
	float e;

	n.xy = gl_PointCoord * 2.0 - 1.0;
	x = dot(n.xy, n.xy);
	n.z = 1.0 - x;
	if(n.z < -0.05) discard;

	e = k1 * clm[8] * (n.x * n.x - n.y * n.y) + k3 * clm[6] * n.z * n.z + k4 * clm[0] - k5 * clm[6]
	    + 2 * k1 * (clm[4] * n.x * n.y + clm[7] * n.x * n.z + clm[5] * n.y * n.z)
		+ 2 * k2 * (clm[3] * n.x + clm[1] * n.y + clm[2] * n.z);

    gl_FragColor = vec4(e, e, e, 1.0);
}