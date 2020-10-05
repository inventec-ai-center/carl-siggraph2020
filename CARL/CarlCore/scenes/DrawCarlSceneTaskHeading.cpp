#include "DrawCarlSceneTaskHeading.h"
#include <math.h>
#include "render/DrawCharacter.h"
#include "render/DrawUtilExtend.h"
#include "sim/SimBox.h"
#include "scenes/SceneSimChar.h"
#include "util/json/json.h"
#include "util/MathUtilExtend.h"
#include "util/FileUtilExtend.h"
#include "util/JsonUtil.h"

extern int g_ScreenWidth;
extern int g_ScreenHeight;

const tVector gCurrentHeadingArrowColor = tVector(1, 0, 0, 0.5);
const tVector gTargetHeadingArrowColor = tVector(0, 1, 0, 0.5);
const double gMoveSpeed = 0.1;

cDrawCarlSceneTaskHeading::cDrawCarlSceneTaskHeading()
{
	mCtrlMode = CtrlMode::eManual;
	mEnableRandomHeading = false;
}

cDrawCarlSceneTaskHeading::~cDrawCarlSceneTaskHeading()
{
}

void cDrawCarlSceneTaskHeading::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawSceneSimChar::ParseArgs(parser);
	cDrawCarlRLScene::ParseArgs(parser);

	std::string path_filename;
	parser->ParseString("path_filename", path_filename);
	std::vector<tVector> ctrl_ptrs = cPathFollower::ParsePathCtrlPoints(path_filename);
	mPathFollower.Init(ctrl_ptrs);
}

void cDrawCarlSceneTaskHeading::Init()
{
	cDrawSceneSimChar::Init();
	cRLScene::Init();

	cCarlSceneTaskHeading* scene = dynamic_cast<cCarlSceneTaskHeading*>(mScene.get());
	scene->EnableRandHeading(mEnableRandomHeading);
}

void cDrawCarlSceneTaskHeading::Clear()
{
	cDrawSceneSimChar::Clear();
	cDrawCarlRLScene::Clear();
	mPathFollower.Clear();
}

bool cDrawCarlSceneTaskHeading::IsEpisodeEnd() const
{
	return cDrawCarlRLScene::IsEpisodeEnd();
}

bool cDrawCarlSceneTaskHeading::CheckValidEpisode() const
{
	return cDrawCarlRLScene::CheckValidEpisode();
}

void cDrawCarlSceneTaskHeading::Update(double time_elapsed)
{
	cDrawSceneSimChar::Update(time_elapsed);

	if (mCtrlMode == CtrlMode::ePathFollow)
	{
		mPathFollower.Update(mScene->GetCharPos());
		cCarlSceneTaskHeading* scene = dynamic_cast<cCarlSceneTaskHeading*>(mScene.get());
		double heading = CalcHeading();
		scene->SetTargetHeading(heading);
	}
}

void cDrawCarlSceneTaskHeading::Keyboard(unsigned char key, double device_x, double device_y)
{
	switch (key)
	{
	case 's':
		ToggleCtrlMode();
		break;
	case 'q':
		MoveTargetHeading(gMoveSpeed * 5);
		break;
	case 'e':
		MoveTargetHeading(-gMoveSpeed * 5);
		break;
	case 'a':
		MoveTargetHeading(gMoveSpeed);
		break;
	case 'd':
		MoveTargetHeading(-gMoveSpeed);
		break;
	default:
		cDrawSceneSimChar::Keyboard(key, device_x, device_y);
		break;
	}
}

std::string cDrawCarlSceneTaskHeading::GetName() const
{
	return cDrawCarlRLScene::GetName();
}

cRLScene* cDrawCarlSceneTaskHeading::GetRLScene() const
{
	return dynamic_cast<cRLScene*>(mScene.get());
}

cCarlRLScene* cDrawCarlSceneTaskHeading::GetCarlRLScene() const
{
	return dynamic_cast<cCarlRLScene*>(mScene.get());
}

void cDrawCarlSceneTaskHeading::ToggleCtrlMode()
{
	mCtrlMode = (CtrlMode)((mCtrlMode + 1) % CtrlMode::eMaxMode);
	if (mCtrlMode == CtrlMode::eManual)
	{
		printf("Ctrl Mode: Manual\n");
	}
	else if (mCtrlMode == CtrlMode::eRandom)
	{
		printf("Ctrl Mode: Random\n");
	}
	else if (mCtrlMode == CtrlMode::ePathFollow)
	{
		printf("Ctrl Mode: Path Following\n");
	}

	cCarlSceneTaskHeading* scene = dynamic_cast<cCarlSceneTaskHeading*>(mScene.get());
	mEnableRandomHeading = (mCtrlMode == CtrlMode::eRandom);
	scene->EnableRandHeading(mEnableRandomHeading);
}

void cDrawCarlSceneTaskHeading::MoveTargetHeading(double delta)
{
	cCarlSceneTaskHeading* scene = dynamic_cast<cCarlSceneTaskHeading*>(mScene.get());
	double heading = scene->GetTargetHeading();
	heading += delta;
	heading = cMathUtilExtend::ClampEuler(heading);
	scene->SetTargetHeading(heading);
}

void cDrawCarlSceneTaskHeading::DrawInfo() const
{
	DrawInfoText();
}

void cDrawCarlSceneTaskHeading::DrawInfoText() const
{
	cDrawUtilExtend::BeginDrawString();
	{
		std::string mode_str = "";
		if (mCtrlMode == CtrlMode::eManual)
		{
			mode_str = "Manual";
		}
		else if (mCtrlMode == CtrlMode::eRandom)
		{
			mode_str = "Random";
		}
		else if (mCtrlMode == CtrlMode::ePathFollow)
		{
			mode_str = "FollowPath";
		}

		std::shared_ptr<cCarlSceneTaskHeading> scene = std::dynamic_pointer_cast<cCarlSceneTaskHeading>(mScene);
		double reward = scene->CalcReward(0);
		double target_heading = scene->GetTargetHeading();

		char str[64];
		float screen_y = g_ScreenHeight * 0.95;
		float screen_x = g_ScreenHeight * 0.02;
		sprintf(str, "Mode: %s\nTarget heading: %.3f rad\nReward: %.3f", mode_str.c_str(), target_heading, reward);
		cDrawUtilExtend::DrawString(screen_x, screen_y, str);

		const std::shared_ptr<cSimCharacter> sim_char = mScene->GetCharacter();
		tVector v_com = sim_char->CalcCOMVel().normalized();
		double heading = atan2(-v_com[2], v_com[0]);

		sprintf(str, "Heading = %.2f rad", heading);

		tVector world_pos = sim_char->CalcCOM();
		world_pos[1] += 0.4;
		tVector screen_pos = cMathUtilExtend::World2Screen(mCamera, world_pos);
		screen_pos[0] -= g_ScreenWidth * 0.1f;
		cDrawUtilExtend::DrawString(screen_pos[0], screen_pos[1], str, tVector(0, 0, 0, 1));
	}
	cDrawUtilExtend::EndDrawString();
}

void cDrawCarlSceneTaskHeading::DrawMisc() const
{
	if (mEnableTrace)
	{
		DrawTrace();
	}

	if (mCtrlMode == CtrlMode::ePathFollow)
	{
		mPathFollower.Draw();
	}

	cDrawUtilExtend::DrawXyzAxes();
}

void cDrawCarlSceneTaskHeading::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cCarlSceneTaskHeading>(new cCarlSceneTaskHeading());
}

void cDrawCarlSceneTaskHeading::DrawCharacters() const
{
	cDrawSceneSimChar::DrawCharacters();
	DrawHeading();
}

void cDrawCarlSceneTaskHeading::DrawHeading() const
{
	std::shared_ptr<cCarlSceneTaskHeading> scene = std::dynamic_pointer_cast<cCarlSceneTaskHeading>(GetScene());
	const std::shared_ptr<cSimCharacter> character = scene->GetCharacter();
	const double target_heading = scene->GetTargetHeading();
	const double arrow_size = 0.2;
	tVector delta_y = tVector(0, 0.0001, 0, 0);
	tVector root_pos = character->GetRootPos();
	tVector start, end;

	// Draw target heading
	start = tVector(root_pos[0], 0, root_pos[2], 0) + delta_y * 1;
	end = tVector(cos(target_heading), 0, -sin(target_heading), 0) + start;
	cDrawUtil::SetColor(gTargetHeadingArrowColor);
	cDrawUtilExtend::DrawArrow2D(start, end, arrow_size);

	// Draw center-of-mass velocity
	tVector v_com = character->CalcCOMVel();
	v_com[1] = 0;
	v_com[3] = 0;
	start = tVector(root_pos[0], 0, root_pos[2], 0) + delta_y * 2;
	end = v_com + start;
	cDrawUtil::SetColor(gCurrentHeadingArrowColor);
	cDrawUtilExtend::DrawArrow2D(start, end, arrow_size);
}

void cDrawCarlSceneTaskHeading::ResetScene()
{
	cDrawSceneSimChar::ResetScene();
	mPathFollower.Reset();
}

double cDrawCarlSceneTaskHeading::CalcHeading() const
{
	tVector dir = mPathFollower.GetProgressPoint() - mScene->GetCharPos();
	dir = dir.normalized();
	double heading = std::atan2(-dir[2], dir[0]);
	return heading;
}
