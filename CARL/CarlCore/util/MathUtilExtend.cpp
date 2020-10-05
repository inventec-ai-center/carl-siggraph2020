#include "MathUtilExtend.h"


extern int g_ScreenWidth;
extern int g_ScreenHeight;


double cMathUtilExtend::ClampEuler(double theta_in)
{
	double theta_out = theta_in;
	if (theta_out > M_PI)
	{
		theta_out -= 2 * M_PI;
	}
	else if (theta_out <= -M_PI)
	{
		theta_out += 2 * M_PI;
	}
	return theta_out;
}

tVector cMathUtilExtend::World2Screen(const cCamera& camera, const tVector& world_pos)
{
	tVector view_pos = camera.BuildWorldViewMatrix() * tVector(world_pos[0], world_pos[1], world_pos[2], 1);
	tVector proj_pos = camera.BuildProjMatrix() * view_pos;
	tVector clip_pos = tVector(proj_pos) / proj_pos[3];
	tVector screen_pos = tVector((clip_pos[0] + 1) * 0.5 * g_ScreenWidth, (clip_pos[1] + 1) * 0.5 * g_ScreenHeight, 0, 0);
	return screen_pos;
}
