uniform vec2 irradiance;

void main(void)
{
	vec3 n;
	float x;
	float y;

	n.xy = gl_PointCoord * 2.0 - 1.0;
	x = dot(n.xy, n.xy);
	n.z = 1.0 - x;
	if(n.z < 0.0) discard;

	y = 2 * x * x * x - 3 * x * x + 1;

    gl_FragColor = vec4(1.0 * y, 1.0 * y, 1.0 * y, 1.0);
}