#include "DrawCarlSceneTaskImitate.h"
#include "CarlSceneTaskImitate.h"
#include "render/DrawCharacterExtend.h"
#include "render/DrawUtilExtend.h"
#include "render/DrawKinTree.h"
#include "sim/RBDUtil.h"
#include "util/json/json.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "util/MathUtilExtend.h"
#include "render/DrawSimCharacter.h"

extern int g_ScreenWidth;
extern int g_ScreenHeight;

const double gLinkWidth = 0.025f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFilLColor = tVector(0.12f, 0.84f, 0.38f, 1);

#define GAIT_PATTERN_BUFFER_SIZE 1000


cDrawCarlSceneTaskImitate::cDrawCarlSceneTaskImitate()
{
	mDrawKinChar = false;
	mEnableRandomAction = false;
	mEnableSyncRootInUpdate = true;
}

cDrawCarlSceneTaskImitate::~cDrawCarlSceneTaskImitate()
{
}

void cDrawCarlSceneTaskImitate::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cDrawSceneSimChar::ParseArgs(parser);
	cDrawCarlRLScene::ParseArgs(parser);
}

void cDrawCarlSceneTaskImitate::Init()
{
	cDrawSceneSimChar::Init();
	cCarlRLScene::Init();

	InitGaits();

	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());
	scene->EnableRandomKinChar(mEnableRandomAction);
	scene->EnableSyncKinCharRootInUpdate(mEnableSyncRootInUpdate);
}

void cDrawCarlSceneTaskImitate::Clear()
{
	cDrawSceneSimChar::Clear();
	cDrawCarlRLScene::Clear();

	mKinGaits.clear();
	mSimGaits.clear();
}

void cDrawCarlSceneTaskImitate::Update(double timestep)
{
	cDrawSceneSimChar::Update(timestep);
	UpdateGait();
}

bool cDrawCarlSceneTaskImitate::IsEpisodeEnd() const
{
	return cDrawCarlRLScene::IsEpisodeEnd();
}

bool cDrawCarlSceneTaskImitate::CheckValidEpisode() const
{
	return cDrawCarlRLScene::CheckValidEpisode();
}

void cDrawCarlSceneTaskImitate::Keyboard(unsigned char key, double device_x, double device_y)
{
	switch (key)
	{
	case 'k':
		DrawKinChar(!mDrawKinChar);
		break;
	case 's':
		ToggleRandomAction();
		break;
	case 'd':
		ToggleSyncRootPosition();
		break;
	default:
		cDrawSceneSimChar::Keyboard(key, device_x, device_y);
		break;
	}
}

void cDrawCarlSceneTaskImitate::DrawKinChar(bool enable)
{
	mDrawKinChar = enable;
	if (mDrawKinChar)
	{
		printf("Enabled draw kinematic character\n");
	}
	else
	{
		printf("Disabled draw kinematic character\n");
	}
}

std::string cDrawCarlSceneTaskImitate::GetName() const
{
	return cDrawCarlRLScene::GetName();
}

cRLScene* cDrawCarlSceneTaskImitate::GetRLScene() const
{
	return dynamic_cast<cRLScene*>(mScene.get());
}

cCarlRLScene* cDrawCarlSceneTaskImitate::GetCarlRLScene() const
{
	return dynamic_cast<cCarlRLScene*>(mScene.get());
}

void cDrawCarlSceneTaskImitate::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cCarlSceneTaskImitate>(new cCarlSceneTaskImitate());
}

void cDrawCarlSceneTaskImitate::DrawCharacters() const
{
	cDrawSceneSimChar::DrawCharacters();
	if (mDrawKinChar)
	{
		DrawKinCharacters();
	}
}

void cDrawCarlSceneTaskImitate::DrawKinCharacters() const
{
	const auto& kin_char = GetKinChar();
	DrawKinCharacter(kin_char);
}

void cDrawCarlSceneTaskImitate::DrawKinCharacter(const std::shared_ptr<cKinCharacter>& kin_char) const
{
	if (kin_char->HasDrawShapes())
	{
		cDrawCharacter::Draw(*kin_char, 0, tVector(1.25, 0.75, 0.75, 1), gLineColor);
		cDrawCharacterExtend::DrawBones(*kin_char, 0.025f, tVector(1, 0.75, 0.25, 0.75), tVector(1, 0, 0, 1));
	}
	else
	{
		cDrawCharacterExtend::DrawBones(*kin_char, gLinkWidth, tVector(0, 0, 0, 1), tVector(1, 0, 0, 1));
	}
}

void cDrawCarlSceneTaskImitate::DrawInfo() const
{
	DrawInfoText();
}

void cDrawCarlSceneTaskImitate::ToggleRandomAction()
{
	mEnableRandomAction = !mEnableRandomAction;
	if (mEnableRandomAction)
	{
		printf("Enable random action\n");
	}
	else
	{
		printf("Disable random action\n");
	}
	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());
	scene->EnableRandomKinChar(mEnableRandomAction);
}

void cDrawCarlSceneTaskImitate::ToggleSyncRootPosition()
{
	mEnableSyncRootInUpdate = !mEnableSyncRootInUpdate;
	if (mEnableSyncRootInUpdate)
	{
		printf("Enable Sync Root Position (Draw only)\n");
	}
	else
	{
		printf("Disable Sync Root Position (Draw only)\n");
	}
	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());
	scene->EnableSyncKinCharRootInUpdate(mEnableSyncRootInUpdate);
}

void cDrawCarlSceneTaskImitate::InitGaits()
{
	mKinGaits.clear();
	mSimGaits.clear();

	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());

	int num_contacts = scene->GetNumContacts();
	for (int i = 0; i < num_contacts; ++i)
	{
		mKinGaits.push_back(std::deque<bool>());
		mSimGaits.push_back(std::deque<bool>());
	}
}

void cDrawCarlSceneTaskImitate::UpdateGait()
{
	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());
	int num_contacts = scene->GetNumContacts();

	const std::shared_ptr<cCarlKinCharacter> kin_char = scene->GetKinChar();

	std::vector<bool> kin_contacts = scene->GetKinCharContacts();
	std::vector<bool> Sim_contacts = scene->GetSimCharContacts(kin_char->GetJointMat());

	for (int i = 0; i < num_contacts; ++i)
	{
		mKinGaits[i].push_back(kin_contacts[i]);
		mSimGaits[i].push_back(Sim_contacts[i]);

		if (mKinGaits[i].size() > GAIT_PATTERN_BUFFER_SIZE)
		{
			mKinGaits[i].pop_front();
			mSimGaits[i].pop_front();
		}
	}
}

void cDrawCarlSceneTaskImitate::DrawInfoText() const
{
	const std::shared_ptr<cSimCharacter> sim_char = mScene->GetCharacter();
	cCarlSceneTaskImitate* scene = dynamic_cast<cCarlSceneTaskImitate*>(mScene.get());
	const std::shared_ptr<cKinCharacter> kin_char = scene->GetKinChar();
	char str[128];

	cDrawUtilExtend::BeginDrawString();
	{
		double reward = scene->CalcReward(0);
		sprintf(str, "Reward: %.3f", reward);
		float screen_y = g_ScreenHeight * 0.95;
		float screen_x = g_ScreenHeight * 0.02;
		cDrawUtilExtend::DrawString(screen_x, screen_y, str, tVector(0, 0, 0, 1));
	}
	cDrawUtilExtend::EndDrawString();

	cDrawUtilExtend::BeginDrawString();
	{
		tVector v_com = sim_char->CalcCOMVel();
		tVector world_pos = sim_char->CalcCOM();
		world_pos[1] += 0.5;
		float speed = v_com.norm();
		sprintf(str, "Speed = %.2f m/s", speed);
		tVector screen_pos = cMathUtilExtend::World2Screen(mCamera, world_pos);
		screen_pos[0] -= g_ScreenWidth * 0.1f;
		cDrawUtilExtend::DrawString(screen_pos[0], screen_pos[1], str, tVector(0, 0, 0, 1));
	}
	cDrawUtilExtend::EndDrawString();

	if (mDrawKinChar)
	{
		cDrawUtilExtend::BeginDrawString();
		{
			const Eigen::VectorXd& kin_pose = kin_char->GetPose();
			const Eigen::VectorXd& kin_vel = kin_char->GetVel();
			tVector com1_world;
			tVector com_vel1_world;
			cRBDUtil::CalcCoM(sim_char->GetJointMat(), sim_char->GetBodyDefs(), kin_pose, kin_vel, com1_world, com_vel1_world);
			float speed = com_vel1_world.norm();
			com1_world[1] += 0.6;
			sprintf(str, "Speed = %.2f m/s", speed);
			tVector screen_pos = cMathUtilExtend::World2Screen(mCamera, com1_world);
			screen_pos[0] -= g_ScreenWidth * 0.1f;
			cDrawUtilExtend::DrawString(screen_pos[0], screen_pos[1], str, tVector(0.75, 0, 0, 1));
		}
		cDrawUtilExtend::EndDrawString();
	}
}

void cDrawCarlSceneTaskImitate::DrawGaitPatternInfo() const
{
	int num_val = static_cast<int>(mKinGaits[0].size());
	double aspect = mCamera.GetAspectRatio();

#if 0
	const double h = 0.4;
	const double w = 16.0 / 9 * h / aspect;
	tVector origin = tVector::Zero();
	origin[0] = 1 - w * 1.05;
	origin[1] = 0.95 - h * 1.05;
	origin[2] = -1;
#else
	const double h = 0.3;
	const double w = 1.94;
	tVector origin = tVector::Zero();
	origin[0] = -0.97;
	origin[1] = -0.95;
	origin[2] = -1;
#endif

	int capacity = GAIT_PATTERN_BUFFER_SIZE;

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0));

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetPointSize(2);
	cDrawUtil::SetColor(tVector(1, 0, 0, 0.75));

	const double h_step = h / (double)mKinGaits.size();

	if (num_val > 0)
	{
		if (mDrawKinChar)
		{
			for (int i = 0; i < mKinGaits.size(); ++i)
			{
				cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));

				double prev_val = mKinGaits[i][0] ? 1 : 0;

				int rect_start_j = -1;
				if (prev_val > 0)
				{
					rect_start_j = 0;
				}

				for (int j = 1; j < num_val; ++j)
				{
					double curr_val = mKinGaits[i][j] ? 1 : 0;

					if (rect_start_j >= 0 && ((curr_val != prev_val) || (j == num_val - 1)))
					{
						tVector rect_origin = tVector::Zero();
						tVector rect_size = tVector::Zero();

						rect_size[0] = w * (j - rect_start_j) / (capacity - 1.0);
						rect_size[1] = h_step;

						rect_origin[0] = w * rect_start_j / (capacity - 1.0) + rect_size[0] * 0.5;
						rect_origin[1] = h_step * i + h_step - rect_size[1] * 0.5;
						rect_origin += origin;

						cDrawUtil::DrawRect(rect_origin, rect_size);

						rect_start_j = -1;
					}
					else if (rect_start_j < 0 && curr_val != prev_val)
					{
						rect_start_j = j;
					}

					prev_val = curr_val;
				}

				if (i > 0)
				{
					cDrawUtil::SetColor(tVector(0, 0, 0, 1.0f));
					cDrawUtil::DrawLine(origin + tVector(0, h_step * i, 0, 0), origin + tVector(w, h_step * i, 0, 0));
				}
			}
		}

		for (int i = 0; i < mSimGaits.size(); ++i)
		{
			cDrawUtil::SetColor(tVector(0, 1, 1, 0.5f));

			double prev_val = mSimGaits[i][0] ? 1 : 0;

			int rect_start_j = -1;
			if (prev_val > 0)
			{
				rect_start_j = 0;
			}

			for (int j = 1; j < num_val; ++j)
			{
				double curr_val = mSimGaits[i][j] ? 1 : 0;

				if (rect_start_j >= 0 && ((curr_val != prev_val) || (j == num_val - 1)))
				{
					tVector rect_origin = tVector::Zero();
					tVector rect_size = tVector::Zero();

					rect_size[0] = w * (j - rect_start_j) / (capacity - 1.0);
					rect_size[1] = h_step;

					rect_origin[0] = w * rect_start_j / (capacity - 1.0) + rect_size[0] * 0.5;
					rect_origin[1] = h_step * i + h_step - rect_size[1] * 0.5;
					rect_origin += origin;

					cDrawUtil::DrawRect(rect_origin, rect_size);

					rect_start_j = -1;
				}
				else if (rect_start_j < 0 && curr_val != prev_val)
				{
					rect_start_j = j;
				}

				prev_val = curr_val;
			}

			if (i > 0)
			{
				cDrawUtil::SetColor(tVector(0, 0, 0, 0.5));
				cDrawUtil::DrawLine(origin + tVector(0, h_step * i, 0, 0), origin + tVector(w, h_step * i, 0, 0));
			}
		}
	}

	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0), cDrawUtil::eDrawWireSimple);
}

const std::shared_ptr<cKinCharacter>& cDrawCarlSceneTaskImitate::GetKinChar() const
{
	const cCarlSceneTaskImitate* scene = dynamic_cast<const cCarlSceneTaskImitate*>(mScene.get());
	return scene->GetKinChar();
}