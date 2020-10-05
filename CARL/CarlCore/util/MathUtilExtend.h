#pragma once

#include "util/MathUtil.h"
#include "render/Camera.h"


class cMathUtilExtend
{
public:

    static double ClampEuler(double theta_in);
    static tVector World2Screen (const cCamera& camera, const tVector& world_pos);

};
