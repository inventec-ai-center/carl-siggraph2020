#include "DrawCarlSceneTaskSpeed.h"
#include <math.h>
#include "CarlSceneTaskSpeed.h"
#include "render/DrawCharacter.h"
#include "render/DrawUtilExtend.h"
#include "sim/DeepMimicCharController.h"
#include "scenes/SceneSimChar.h"
#include "util/json/json.h"
#include "util/FileUtilExtend.h"
#include "util/JsonUtil.h"
#include "util/MathUtilExtend.h"

extern int g_ScreenWidth;
extern int g_ScreenHeight;

const tVector gCurrentVelocityArrowColor = tVector(1, 0, 0, 0.5);
const tVector gTargetVelocityArrowColor = tVector(0, 1, 0, 0.5);
const double gMoveSpeed = 0.05;

cDrawCarlSceneTaskSpeed::cDrawCarlSceneTaskSpeed()
{
	mCtrlMode = CtrlMode::eManual;
	mEnableRandomVelocity = false;
}

cDrawCarlSceneTaskSpeed::~cDrawCarlSceneTaskSpeed()
{
}

void cDrawCarlSceneTaskSpeed::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawSceneSimChar::ParseArgs(parser);
	cDrawCarlRLScene::ParseArgs(parser);
}

void cDrawCarlSceneTaskSpeed::Init()
{
	cDrawSceneSimChar::Init();
	cDrawCarlRLScene::Init();

	cCarlSceneTaskSpeed* scene = dynamic_cast<cCarlSceneTaskSpeed*>(mScene.get());
	scene->EnableRandVelocity(mEnableRandomVelocity);
	scene->SetTargetVelocity(1.0);
}

void cDrawCarlSceneTaskSpeed::Clear()
{
	cDrawSceneSimChar::Clear();
	cDrawCarlRLScene::Clear();
}

bool cDrawCarlSceneTaskSpeed::IsEpisodeEnd() const
{
	return cDrawCarlRLScene::IsEpisodeEnd();
}

bool cDrawCarlSceneTaskSpeed::CheckValidEpisode() const
{
	return cDrawCarlRLScene::CheckValidEpisode();
}

void cDrawCarlSceneTaskSpeed::Update(double time_elapsed)
{
	cDrawSceneSimChar::Update(time_elapsed);
}

void cDrawCarlSceneTaskSpeed::Keyboard(unsigned char key, double device_x, double device_y)
{
	switch (key)
	{
	case 's':
		ToggleCtrlMode();
		break;
	case 'a':
		AddTargetVelocity(gMoveSpeed);
		break;
	case 'd':
		AddTargetVelocity(-gMoveSpeed);
		break;
	case 'q':
		AddTargetVelocity(gMoveSpeed * 10);
		break;
	case 'e':
		AddTargetVelocity(-gMoveSpeed * 10);
		break;
	default:
		cDrawSceneSimChar::Keyboard(key, device_x, device_y);
		break;
	}
}

std::string cDrawCarlSceneTaskSpeed::GetName() const
{
	return cDrawCarlRLScene::GetName();
}

cRLScene* cDrawCarlSceneTaskSpeed::GetRLScene() const
{
	return dynamic_cast<cRLScene*>(mScene.get());
}

cCarlRLScene* cDrawCarlSceneTaskSpeed::GetCarlRLScene() const
{
	return dynamic_cast<cCarlRLScene*>(mScene.get());
}

void cDrawCarlSceneTaskSpeed::ToggleCtrlMode()
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

	cCarlSceneTaskSpeed* scene = dynamic_cast<cCarlSceneTaskSpeed*>(mScene.get());
	mEnableRandomVelocity = (mCtrlMode == CtrlMode::eRandom);
	scene->EnableRandVelocity(mEnableRandomVelocity);
}

void cDrawCarlSceneTaskSpeed::AddTargetVelocity(double delta)
{
	cCarlSceneTaskSpeed* scene = dynamic_cast<cCarlSceneTaskSpeed*>(mScene.get());
	double velocity = scene->GetTargetVelocity();
	velocity += delta;
	scene->SetTargetVelocity(velocity);
}

void cDrawCarlSceneTaskSpeed::DrawInfo() const
{
	DrawInfoText();
}

void cDrawCarlSceneTaskSpeed::DrawInfoText() const
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

		const std::shared_ptr<cCarlSceneTaskSpeed> vel_scene = std::dynamic_pointer_cast<cCarlSceneTaskSpeed>(mScene);
		double target_velocity = vel_scene->GetTargetVelocity();
		double reward = vel_scene->CalcReward(0);

		char str[64];
		float screen_y = g_ScreenHeight * 0.95;
		float screen_x = g_ScreenHeight * 0.02;
		sprintf(str, "Mode: %s\nTarget speed: %.3f m/s\nReward: %.3f", mode_str.c_str(), target_velocity, reward);
		cDrawUtilExtend::DrawString(screen_x, screen_y, str);

		const std::shared_ptr<cSimCharacter> sim_char = mScene->GetCharacter();
		tVector world_pos = sim_char->CalcCOM();
		world_pos[1] += 0.4;
		float speed = sim_char->CalcCOMVel().norm();
		sprintf(str, "Speed = %.2f m/s", speed);
		tVector screen_pos = cMathUtilExtend::World2Screen(mCamera, world_pos);
		screen_pos[0] -= g_ScreenWidth * 0.1f;
		cDrawUtilExtend::DrawString(screen_pos[0], screen_pos[1], str, tVector(0, 0, 0, 1));
	}
	cDrawUtilExtend::EndDrawString();
}

void cDrawCarlSceneTaskSpeed::DrawMisc() const
{
	if (mEnableTrace)
	{
		DrawTrace();
	}

	cDrawUtilExtend::DrawXyzAxes();
}

void cDrawCarlSceneTaskSpeed::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cCarlSceneTaskSpeed>(new cCarlSceneTaskSpeed());
}

void cDrawCarlSceneTaskSpeed::DrawCharacters() const
{
	cDrawSceneSimChar::DrawCharacters();
	DrawHeading();
}

void cDrawCarlSceneTaskSpeed::DrawHeading() const
{
	std::shared_ptr<cCarlSceneTaskSpeed> scene = std::dynamic_pointer_cast<cCarlSceneTaskSpeed>(GetScene());
	const std::shared_ptr<cSimCharacter> character = scene->GetCharacter();
	const double target_heading = scene->GetTargetHeading();
	const double current_heading = character->CalcHeading();
	const double target_velocity = scene->GetTargetVelocity();
	const double current_velocity = character->CalcCOMVel().norm();
	const double arrow_size = 0.2;
	tVector delta_y = tVector(0, 0.0001, 0, 0);
	tVector root_pos = character->GetRootPos();
	tVector start, end;

	// Draw target velocity
	start = tVector(root_pos[0], 0, root_pos[2], 0) + delta_y * 2;
	end = tVector(cos(target_heading), 0, -sin(target_heading), 0) * sqrt(target_velocity) + start;
	cDrawUtil::SetColor(gTargetVelocityArrowColor);
	cDrawUtilExtend::DrawArrow2D(start, end, arrow_size);

	// Draw center-of-mass velocity
	tVector v_com_norm = character->CalcCOMVel().normalized();
	v_com_norm[1] = 0;
	v_com_norm[3] = 0;
	start = tVector(root_pos[0], 0, root_pos[2], 0) + delta_y * 3;
	end = v_com_norm * sqrt(current_velocity) + start;
	cDrawUtil::SetColor(gCurrentVelocityArrowColor);
	cDrawUtilExtend::DrawArrow2D(start, end, arrow_size);
}
