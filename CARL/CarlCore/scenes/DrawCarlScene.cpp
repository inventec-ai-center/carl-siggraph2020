#include "DrawCarlScene.h"
#include "util/MathUtil.h"

// camera attributes
const tVector gCameraPosition = tVector(20, 0, 0, 0);
const tVector gCameraFocus = tVector(0, gCameraPosition[1], gCameraPosition[2], 0.0);
const tVector gCameraUp = tVector(0, 1, 0, 0);
const double gViewWidth = 12;
const double gViewHeight = gViewWidth * 9.0 / 16.0;
const double gViewNearZ = 0.1;
const double gViewFarZ = 500;

extern int g_ScreenWidth;
extern int g_ScreenHeight;

cDrawCarlScene::cDrawCarlScene() : cDrawScene::cDrawScene()
{
    mCamPosition = gCameraPosition;
	mCamUp = gCameraUp;
	mCamZoom = 0;
}

cDrawCarlScene::~cDrawCarlScene()
{
}

void cDrawCarlScene::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
    cDrawScene::ParseArgs(parser);
	
    ParseCamView(parser);
}

void cDrawCarlScene::ParseCamView(const std::shared_ptr<cArgParser>& parser)
{
	std::vector<double> values;
	parser->ParseDoubles("cam_position", values);
	if (values.size() >= 3)
	{
		mCamPosition[0] = values[0];
		mCamPosition[1] = values[1];
		mCamPosition[2] = values[2];
		mCamPosition = mCamPosition.normalized() * (gCameraPosition - gCameraFocus).norm();
	}
	parser->ParseDoubles("cam_up", values);
	if (values.size() >= 3)
	{
		mCamUp[0] = values[0];
		mCamUp[1] = values[1];
		mCamUp[2] = values[2];
	}
	parser->ParseDouble("cam_zoom", mCamZoom);
}

void cDrawCarlScene::InitCamera()
{
    cCamera::eProj proj = cCamera::eProjPerspective;
	mCamera = cCamera(proj, mCamPosition, gCameraFocus, mCamUp, 
                      gViewWidth, gViewHeight, gViewNearZ, gViewFarZ);

	// Hack: Set camera zoom
	const float zoom_level = 0.05f;
	int n_steps = ceil(mCamZoom / zoom_level);
	for (int i = 0; i < n_steps; ++i) 
	{
		if (zoom_level > 0)
		{
			mCamera.MouseClick(4, 0, 0, 0);
		}
		else 
		{
			mCamera.MouseClick(3, 0, 0, 0);
		}
	}
}

void cDrawCarlScene::Reshape(int w, int h)
{
	cDrawScene::Reshape(w, h);

	g_ScreenWidth = w;
	g_ScreenHeight = h;
}
