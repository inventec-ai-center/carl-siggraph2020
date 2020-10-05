#pragma once

#include "render/DrawUtil.h"

#define CHECK_GL_ERROR cDrawUtilExtend::CheckOpenGLError(__LINE__, __FILE__);

class cDrawUtilExtend
{
public:

	static void InitDrawUtilExtend();
	static void BeginDrawString();
	static void EndDrawString();
	static void DrawString(float x, float y, char* string, tVector color = tVector(0, 0, 0, 1));
	static void BuildTexts();
	static void BuildMeshes();
	static void DrawArrow2D(const tVector& start, const tVector& end, double head_size);
	static void DrawXyzAxes();
	static void DrawBone(const tVector& pos, const tVector& size, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void Reshape(int w, int h);

	static void CheckOpenGLError(int line, std::string src);

protected:

	static std::unique_ptr<cDrawMesh> gBoneSolidMesh;
	static std::unique_ptr<cDrawMesh> gBoneWireMesh;

	static void DrawBoneSolid(const tVector& pos, const tVector& size);
	static void DrawBoneWire(const tVector& pos, const tVector& size);
};
