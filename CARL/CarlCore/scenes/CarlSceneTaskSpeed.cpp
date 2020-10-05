#include "CarlSceneTaskSpeed.h"
#include <math.h>
#include "sim/RBDUtil.h"
#include "sim/CtController.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "sim/CharController.h"

const int gSpeedCtrlFrameRate = 600;

double cCarlSceneTaskSpeed::CalcRewardVelocity(const cSimCharacter& sim_char, const double target_velocity) const
{
	double v_norm_current = sim_char.CalcCOMVel().norm();
	double v_norm_target  = mTargetVelocity;
	double reward = pow(v_norm_target - v_norm_current, 2);
	reward = exp(-0.8 * reward);
	return reward;
}

cCarlSceneTaskSpeed::cCarlSceneTaskSpeed()
{
	mEnableRandRotReset = false;
	mEnableRootRotFail = false;
	mEnableRandVelocity = true;
	mTargetVelocity = 0;
	mTargetHeading = 0;
	mUpdateCount = 0;
	mMinRandVelocity = 0;
	mMaxRandVelocity = 1;
}

cCarlSceneTaskSpeed::~cCarlSceneTaskSpeed()
{
}

void cCarlSceneTaskSpeed::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cCarlRLSceneSimChar::ParseArgs(parser);

	std::vector<std::string> motion_files;
	parser->ParseStrings("motion_files", motion_files);
	mMotionFile = motion_files[0];
	parser->ParseBool("enable_root_rot_fail", mEnableRootRotFail);
	parser->ParseBool("enable_rand_rot_reset", mEnableRandRotReset);
	parser->ParseDouble("task_min_speed", mMinRandVelocity);
	parser->ParseDouble("task_max_speed", mMaxRandVelocity);
}

void cCarlSceneTaskSpeed::Init()
{
	BuildKinChars();
	cCarlRLSceneSimChar::Init();
}

const std::shared_ptr<cKinCharacter>& cCarlSceneTaskSpeed::GetKinChar() const
{
	return mKinChar;
}

void cCarlSceneTaskSpeed::EnableRandRotReset(bool enable)
{
	mEnableRandRotReset = enable;
}

bool cCarlSceneTaskSpeed::EnabledRandRotReset() const
{
	bool enable = mEnableRandRotReset;
	return enable;
}

void cCarlSceneTaskSpeed::EnableRandVelocity(bool enable)
{
	mEnableRandVelocity = enable;
}

void cCarlSceneTaskSpeed::SetTargetVelocity(double velocity)
{
	mTargetVelocity = velocity;
	mTargetVelocity = cMathUtil::Clamp(mTargetVelocity, mMinRandVelocity, mMaxRandVelocity);
}

double cCarlSceneTaskSpeed::GetTargetVelocity() const
{
	return mTargetVelocity;
}

double cCarlSceneTaskSpeed::GetTargetHeading() const
{
	return mTargetHeading;
}

double cCarlSceneTaskSpeed::CalcReward(int agent_id) const
{
	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	bool fallen = HasFallen(*sim_char);

	double r = 0;
	int max_id = 0;
	if (!fallen)
	{
		r = CalcRewardVelocity(*sim_char, mTargetVelocity);
	}
	return r;
}

void cCarlSceneTaskSpeed::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int goal_size = GetGoalSize(agent_id);
	out_goal = Eigen::VectorXd(goal_size);
	out_goal.setZero();

	tVector velocity = GetCharacter(0)->CalcCOMVel();
	out_goal(0) = velocity.norm();
	out_goal(1) = 0;
	out_goal(2) = mTargetVelocity;
}

void cCarlSceneTaskSpeed::RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const
{
	out_goal = Eigen::VectorXd(1);
	out_goal.setZero();

	tVector velocity = GetCharacter(0)->CalcCOMVel();
	out_goal(0) = velocity.norm();
}

int cCarlSceneTaskSpeed::GetGoalSize(int agent_id) const
{
	return 3;
}

void cCarlSceneTaskSpeed::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int goal_size = GetGoalSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(goal_size);
	out_scale = Eigen::VectorXd::Ones(goal_size);
}

void cCarlSceneTaskSpeed::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int goal_size = GetGoalSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(goal_size);
}

std::string cCarlSceneTaskSpeed::GetName() const
{
	return "Task Velocity Control";
}

bool cCarlSceneTaskSpeed::BuildCharacters()
{
	bool succ = cCarlRLSceneSimChar::BuildCharacters();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
	return succ;
}

void cCarlSceneTaskSpeed::BuildKinChars()
{
	bool succ = BuildKinCharacter(0, mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}
}

bool cCarlSceneTaskSpeed::BuildKinCharacter(int id, std::shared_ptr<cKinCharacter>& out_char) const
{
	auto kin_char = std::shared_ptr<cKinCharacter>(new cKinCharacter());
	const cSimCharacter::tParams& sim_char_params = mCharParams[0];
	cKinCharacter::tParams kin_char_params;

	kin_char_params.mID = id;
	kin_char_params.mCharFile = sim_char_params.mCharFile;
	kin_char_params.mOrigin = sim_char_params.mInitPos;
	kin_char_params.mLoadDrawShapes = false;
	kin_char_params.mMotionFile = mMotionFile;

	bool succ = kin_char->Init(kin_char_params);
	if (succ)
	{
		out_char = kin_char;
	}
	return succ;
}

void cCarlSceneTaskSpeed::UpdateCharacters(double timestep)
{
	mUpdateCount++; // 600 FPS
	if (mEnableRandVelocity && mUpdateCount >= 150) // 4 FPS (0.25 sec)
	{
		mUpdateCount = 0;
		UpdateTargetVelocity(timestep);
	}

	cCarlRLSceneSimChar::UpdateCharacters(timestep);
}

void cCarlSceneTaskSpeed::UpdateTargetVelocity(double timestep)
{
	mTargetVelocity += cMathUtil::RandDouble(-0.25, 0.25);
	mTargetVelocity = cMathUtil::Clamp(mTargetVelocity, mMinRandVelocity, mMaxRandVelocity);
}

double cCarlSceneTaskSpeed::GetKinTime() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->GetTime();
}

void cCarlSceneTaskSpeed::SyncCharacters()
{
	const auto& sim_char = GetCharacter();

	const auto& kin_char = GetKinChar();
	const Eigen::VectorXd& pose = kin_char->GetPose();
	const Eigen::VectorXd& vel = kin_char->GetVel();

	sim_char->SetPose(pose);
	sim_char->SetVel(vel);

	const auto& ctrl = sim_char->GetController();
	auto ct_ctrl = dynamic_cast<cCtController*>(ctrl.get());
	if (ct_ctrl != nullptr)
	{
		double kin_time = GetKinTime();
		ct_ctrl->SetInitTime(kin_time);
	}
}

bool cCarlSceneTaskSpeed::EnableSyncChar() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->HasMotion();
}

void cCarlSceneTaskSpeed::ResetCharacters()
{
	cCarlRLSceneSimChar::ResetCharacters();

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}

	if (mEnableRandVelocity)
	{
		mTargetVelocity = cMathUtil::RandDouble(mMinRandVelocity, mMaxRandVelocity);
	}
}

void cCarlSceneTaskSpeed::ResetKinChar()
{
	const cSimCharacter::tParams& char_params = mCharParams[0];
	const auto& kin_char = GetKinChar();
	double rand_time = CalcRandKinResetTime();

	kin_char->Reset();
	kin_char->SetOriginRot(tQuaternion::Identity());
	kin_char->SetOriginPos(char_params.mInitPos); // reset origin
	kin_char->SetTime(rand_time);
	kin_char->Pose(rand_time);

	if (EnabledRandRotReset())
	{
		double rand_theta = mRand.RandDouble(-M_PI, M_PI);
		kin_char->RotateOrigin(cMathUtil::EulerToQuaternion(tVector(0, rand_theta, 0, 0)));
		mTargetHeading = rand_theta;
	}
}

double cCarlSceneTaskSpeed::CalcRandKinResetTime()
{
	const auto& kin_char = GetKinChar();
	double dur = kin_char->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	return rand_time;
}
