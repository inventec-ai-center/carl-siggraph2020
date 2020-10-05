#include "CarlCore.h"
#include "scenes/CarlSceneKinChar.h"
#include "scenes/CarlSceneTaskImitate.h"
#include "scenes/CarlSceneTaskSpeed.h"
#include "scenes/CarlSceneTaskHeading.h"
#include "scenes/DrawCarlSceneKinChar.h"
#include "scenes/DrawCarlSceneTaskImitate.h"
#include "scenes/DrawCarlSceneTaskSpeed.h"
#include "scenes/DrawCarlSceneTaskHeading.h"
#include "render/DrawUtilExtend.h"


cCarlCore::cCarlCore(bool enable_draw) : cDeepMimicCore(enable_draw)
{

}

cCarlCore::~cCarlCore()
{

}

void cCarlCore::Init()
{
	if (EnableDraw())
	{
		cDrawUtil::InitDrawUtil();
		cDrawUtilExtend::InitDrawUtilExtend();
		InitFrameBuffer();
	}
	SetupScene();
}

void cCarlCore::Reshape(int w, int h)
{
	cDeepMimicCore::Reshape(w, h);
	cDrawUtilExtend::Reshape(w, h);
}

bool cCarlCore::IsCarlRLScene() const
{
	const auto& carl_rl_scene = GetCarlRLScene();
	return carl_rl_scene != nullptr;
}

std::vector<double> cCarlCore::RecordGoalTarget(int agent_id) const
{
	const auto& carl_rl_scene = GetCarlRLScene();
	if (carl_rl_scene != nullptr)
	{
		Eigen::VectorXd goal;
		carl_rl_scene->RecordGoalTarget(agent_id, goal);

		std::vector<double> out_goal;
		ConvertVector(goal, out_goal);
		return out_goal;
	}
	return std::vector<double>(0);
}

void cCarlCore::LogGatingWeights(int agent_id, const std::vector<double>& weights)
{

}

void cCarlCore::LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds)
{

}

void cCarlCore::SetupScene()
{
	ClearScene();

	std::string scene_name = "";
	mArgParser->ParseString("scene", scene_name);
	if (scene_name == "")
	{
		printf("No scene specified\n");
		assert(false);
	}

	mScene = nullptr;
	mRLScene = nullptr;
	mCarlRLScene = nullptr;
	if (EnableDraw())
	{
		if (scene_name == "kin_char")
		{
			mScene = std::shared_ptr<cDrawCarlSceneKinChar>(new cDrawCarlSceneKinChar());
		}
		else if (scene_name == "task_imitate")
		{
			mScene = std::shared_ptr<cDrawCarlSceneTaskImitate>(new cDrawCarlSceneTaskImitate());
		}
		else if (scene_name == "task_speed")
		{
			mScene = std::shared_ptr<cDrawCarlSceneTaskSpeed>(new cDrawCarlSceneTaskSpeed());
		}
		else if (scene_name == "task_heading")
		{
			mScene = std::shared_ptr<cDrawCarlSceneTaskHeading>(new cDrawCarlSceneTaskHeading());
		}
		else
		{
			printf("Unsupported draw scene: %s\n", scene_name.c_str());
			assert(false);
		}
	}
	else
	{
		if (scene_name == "kin_char")
		{
			mScene = std::shared_ptr<cCarlSceneKinChar>(new cCarlSceneKinChar());
		}
		else if (scene_name == "task_imitate")
		{
			mScene = std::shared_ptr<cCarlSceneTaskImitate>(new cCarlSceneTaskImitate());
		}
		else if (scene_name == "task_speed")
		{
			mScene = std::shared_ptr<cCarlSceneTaskSpeed>(new cCarlSceneTaskSpeed());
		}
		else if (scene_name == "task_heading")
		{
			mScene = std::shared_ptr<cCarlSceneTaskHeading>(new cCarlSceneTaskHeading());
		}
		else
		{
			printf("Unsupported scene: %s\n", scene_name.c_str());
			assert(false);
		}
	}

	if (mScene != nullptr)
	{
		mRLScene = std::dynamic_pointer_cast<cRLScene>(mScene);
		mCarlRLScene = std::dynamic_pointer_cast<cCarlRLScene>(mScene);
		mScene->ParseArgs(mArgParser);
		mScene->Init();
		printf("Loaded scene: %s\n", mScene->GetName().c_str());
	}
}

const std::shared_ptr<cCarlRLScene>& cCarlCore::GetCarlRLScene() const
{
	return mCarlRLScene;
}
