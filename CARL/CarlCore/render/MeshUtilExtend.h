#pragma once

#include "render/MeshUtil.h"


class cMeshUtilExtend
{
public:

    static void BuildBoneSolidMesh(std::unique_ptr<cDrawMesh>& out_mesh);
	static void BuildBoneWireMesh(std::unique_ptr<cDrawMesh>& out_mesh);

protected:

};
