#include "DrawUtilExtend.h"
#include "render/MeshUtilExtend.h"

int g_ScreenWidth = 800;
int g_ScreenHeight = static_cast<int>(g_ScreenWidth * 9.0 / 16.0);


std::unique_ptr<cDrawMesh> cDrawUtilExtend::gBoneSolidMesh = nullptr;
std::unique_ptr<cDrawMesh> cDrawUtilExtend::gBoneWireMesh = nullptr;


void cDrawUtilExtend::InitDrawUtilExtend()
{
	BuildMeshes();
}

void cDrawUtilExtend::BeginDrawString()
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glLineWidth(1.5f);
	glColor4f(0.0f, 0.0f, 0.0f, 1.0f);

	cDrawUtil::UnbindDefaultProg();
}

void cDrawUtilExtend::EndDrawString()
{
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	cDrawUtil::BindDefaultProg();
}

void cDrawUtilExtend::DrawString(float x, float y, char* string, tVector color)
{
	const float aspect = (float)g_ScreenWidth / g_ScreenHeight;
	const float text_size = 0.09f;
	const tVector scale = tVector(text_size / aspect, text_size, 1, 0);

	std::istringstream str_stream(string);
	std::string curr_str = "";

	glPushMatrix();
	const float default_scale = 0.0045f;

	float _x = ((x / g_ScreenWidth) - 0.5f) * 2.0f;
	float _y = ((y / g_ScreenHeight) - 0.5f) * 2.0f;

	glTranslatef(_x, _y, 0);
	glScalef(default_scale * scale[0], default_scale * scale[1], scale[2]);

	while (std::getline(str_stream, curr_str))
	{
		glPushMatrix();
		for (size_t i = 0; i < curr_str.size(); ++i)
		{
			glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, curr_str[i]);
		}
		glPopMatrix();

		glTranslatef(0, -1 / default_scale, 0);
	}
	glPopMatrix();
}

void cDrawUtilExtend::BuildTexts()
{

}

void cDrawUtilExtend::BuildMeshes()
{
	cMeshUtilExtend::BuildBoneSolidMesh(gBoneSolidMesh);
	cMeshUtilExtend::BuildBoneWireMesh(gBoneWireMesh);
}

void cDrawUtilExtend::DrawArrow2D(const tVector& start, const tVector& end, double head_size)
{
	GLboolean prev_enable;
	glGetBooleanv(GL_CULL_FACE, &prev_enable);
	glDisable(GL_CULL_FACE);

	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}

	dir[3] = 0;
	tVector axis = tVector(0, 1, 0, 0);
	tVector tangent = axis.cross3(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	tVector body_end = end - dir * head_size;

	tVector a = start - width * tangent;
	tVector b = body_end - width * tangent;
	tVector c = body_end + width * tangent;
	tVector d = start + width * tangent;
	cDrawUtil::DrawQuad(a, b, c, d);

	tVector e0 = body_end - tangent * head_size * 0.5f;
	tVector e1 = body_end + tangent * head_size * 0.5f;
	cDrawUtil::DrawQuad(end, e1, e0, end);

	if (prev_enable)
	{
		glEnable(GL_CULL_FACE);
	}
}

void cDrawUtilExtend::DrawXyzAxes()
{
	const float line_length = 1.0f;
	const float y_offset = 0.005f;

	cDrawUtil::SetColor(tVector(1, 0, 0, 1));
	cDrawUtil::DrawLine(tVector(0, y_offset, 0, 0), tVector(line_length, y_offset, 0, 0));

	cDrawUtil::SetColor(tVector(0, 1, 0, 1));
	cDrawUtil::DrawLine(tVector(0, y_offset, 0, 0), tVector(0, line_length + y_offset, 0, 0));

	cDrawUtil::SetColor(tVector(0, 0, 1, 1));
	cDrawUtil::DrawLine(tVector(0, y_offset, 0, 0), tVector(0, y_offset, line_length, 0));
}

void cDrawUtilExtend::DrawBone(const tVector& pos, const tVector& size, cDrawUtil::eDrawMode draw_mode /*= eDrawSolid*/)
{
	if (draw_mode == cDrawUtil::eDrawWire || draw_mode == cDrawUtil::eDrawWireSimple)
	{
		DrawBoneWire(pos, size);
	}
	else if (draw_mode == cDrawUtil::eDrawSolid)
	{
		DrawBoneSolid(pos, size);
	}
	else
	{
		assert(false); // unsupported draw mode
	}
}

void cDrawUtilExtend::DrawBoneSolid(const tVector& pos, const tVector& size)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(pos);
	cDrawUtil::Scale(size);
	gBoneSolidMesh->Draw(GL_TRIANGLES);
	cDrawUtil::PopMatrixView();
}

void cDrawUtilExtend::DrawBoneWire(const tVector& pos, const tVector& size)
{
	cDrawUtil::PushMatrixView();
	cDrawUtil::Translate(pos);
	cDrawUtil::Scale(size);
	gBoneWireMesh->Draw(GL_LINES);
	cDrawUtil::PopMatrixView();
}

void cDrawUtilExtend::Reshape(int w, int h)
{
	BuildTexts();
}

void cDrawUtilExtend::CheckOpenGLError(int line, std::string src)
{
	GLenum err;
	while ((err = glGetError()) != GL_NO_ERROR)
	{
		const GLubyte *str = gluErrorString(err);
		if (str)
			printf("OpenGL error %d (line %d in %s): %s\n", err, line, src.c_str(), str);
		else
			printf("OpenGL error %d (line %d in %s): NULL\n", err, line, src.c_str());
	}
}
