#include "CarlSceneTaskHeading.h"
#include <math.h>
#include "sim/RBDUtil.h"
#include "sim/CtController.h"
#include "sim/CharController.h"
#include "sim/SimBox.h"
#include "sim/SimSphere.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "util/MathUtilExtend.h"

const int gDirectionCtrlFrameRate = 600;

double cCarlSceneTaskHeading::CalcRewardHeading(const cSimCharacter& sim_char, const double target_heading) const
{
	const double target_speed = 1.0;
	tVector v_com = sim_char.CalcCOMVel();
	v_com = tVector(v_com[0], v_com[2], 0, 0);
	tVector u = tVector::Zero();
	u[0] = cos(target_heading);
	u[1] = -sin(target_heading);
	double reward = 0;
	reward = u.dot(v_com) / (u.norm() * v_com.norm());
	reward = (reward + 1) / 2.0;
	return reward;
}

cCarlSceneTaskHeading::cCarlSceneTaskHeading()
{
	mEnableRandRotReset = false;
	mEnableRootRotFail = false;
	mEnableRandHeading = true;
	mTargetHeading = 0;
	mTargetVelocity = 1;
	mUpdateCount = 0;
}

cCarlSceneTaskHeading::~cCarlSceneTaskHeading()
{
}

void cCarlSceneTaskHeading::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cCarlRLSceneSimChar::ParseArgs(parser);

	std::vector<std::string> motion_files;
	parser->ParseStrings("motion_files", motion_files);
	mMotionFile = motion_files[0];

	parser->ParseBool("enable_root_rot_fail", mEnableRootRotFail);
}

void cCarlSceneTaskHeading::Init()
{
	BuildKinChars();
	cCarlRLSceneSimChar::Init();
}

const std::shared_ptr<cKinCharacter>& cCarlSceneTaskHeading::GetKinChar() const
{
	return mKinChar;
}

void cCarlSceneTaskHeading::EnableRandRotReset(bool enable)
{
	mEnableRandRotReset = enable;
}

bool cCarlSceneTaskHeading::EnabledRandRotReset() const
{
	bool enable = mEnableRandRotReset;
	return enable;
}

void cCarlSceneTaskHeading::EnableRandHeading(bool enable)
{
	mEnableRandHeading = enable;
}

void cCarlSceneTaskHeading::SetTargetHeading(double heading)
{
	mTargetHeading = heading;
}

double cCarlSceneTaskHeading::GetTargetHeading() const
{
	return mTargetHeading;
}

double cCarlSceneTaskHeading::GetTargetVelocity() const
{
	return mTargetVelocity;
}

void cCarlSceneTaskHeading::SetSimCharRotation(double heading)
{
	const auto& sim_char = GetCharacter();
	tQuaternion qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0), -M_PI * 0.5);
	qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), M_PI) * qrot;
	qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), heading) * qrot;
	sim_char->SetRootRotation(qrot);
}

double cCarlSceneTaskHeading::CalcReward(int agent_id) const
{
	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	bool fallen = HasFallen(*sim_char);

	double r = 0;
	int max_id = 0;
	if (!fallen)
	{
		r = CalcRewardHeading(*sim_char, mTargetHeading);
	}
	return r;
}

void cCarlSceneTaskHeading::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	int goal_size = GetGoalSize(agent_id);
	out_goal = Eigen::VectorXd(goal_size);
	out_goal.setZero();

	tVector v_com = GetCharacter()->CalcCOMVel().normalized();
	double theta_curr = atan2(-v_com[2], v_com[0]);
	double theta_delta = cMathUtilExtend::ClampEuler(mTargetHeading - theta_curr);

	out_goal(0) = v_com[0];
	out_goal(1) = v_com[2];
	out_goal(2) = v_com.norm();
	out_goal(3) = theta_delta;
}

void cCarlSceneTaskHeading::RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const
{
	out_goal = Eigen::VectorXd(1);
	out_goal.setZero();
}

int cCarlSceneTaskHeading::GetGoalSize(int agent_id) const
{
	return 4;
}

void cCarlSceneTaskHeading::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int goal_size = GetGoalSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(goal_size);
	out_scale = Eigen::VectorXd::Ones(goal_size);
}

void cCarlSceneTaskHeading::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int goal_size = GetGoalSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(goal_size);
}

std::string cCarlSceneTaskHeading::GetName() const
{
	return "Task Heading";
}

bool cCarlSceneTaskHeading::BuildCharacters()
{
	bool succ = cCarlRLSceneSimChar::BuildCharacters();
	if (EnableSyncChar())
	{
		SyncCharacters(mTargetHeading);
	}
	return succ;
}

void cCarlSceneTaskHeading::BuildKinChars()
{
	bool succ = BuildKinCharacter(0, mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}
}

bool cCarlSceneTaskHeading::BuildKinCharacter(int id, std::shared_ptr<cKinCharacter>& out_char) const
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

void cCarlSceneTaskHeading::UpdateCharacters(double timestep)
{
	mUpdateCount++; // 600 FPS
	if (mEnableRandHeading && mUpdateCount >= 150) // 4 FPS (0.25 sec)
	{
		mUpdateCount = 0;
		UpdateTargetHeading(timestep);
	}

	cCarlRLSceneSimChar::UpdateCharacters(timestep);
}

void cCarlSceneTaskHeading::UpdateTargetHeading(double timestep)
{
	// 10% probability to completely change the heading
	if (cMathUtil::FlipCoin(0.1))
	{
		mTargetHeading = cMathUtil::RandDouble(-M_PI, M_PI);
	}
	else
	{
		mTargetHeading += cMathUtil::RandDouble(-0.15, 0.15);
		mTargetHeading = cMathUtilExtend::ClampEuler(mTargetHeading);
	}
}

double cCarlSceneTaskHeading::GetKinTime() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->GetTime();
}

void cCarlSceneTaskHeading::SyncCharacters(double target_heading)
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

	tQuaternion qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 0, 1, 0), -M_PI * 0.5);
	qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), M_PI) * qrot;
	qrot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), target_heading) * qrot;
	sim_char->SetRootRotation(qrot);
}

bool cCarlSceneTaskHeading::EnableSyncChar() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->HasMotion();
}

void cCarlSceneTaskHeading::ResetCharacters()
{
	cCarlRLSceneSimChar::ResetCharacters();

	if (mEnableRandHeading)
	{
		mTargetHeading = cMathUtil::RandDouble(-M_PI, M_PI);
	}
	else
	{
		auto& sim_char = GetCharacter();
		tVector v_com_norm = sim_char->CalcCOMVel().normalized();
		double heading = atan2(-v_com_norm[2], v_com_norm[0]);
		mTargetHeading = heading;
	}

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters(mTargetHeading);
	}
}

void cCarlSceneTaskHeading::ResetKinChar()
{
	const cSimCharacter::tParams& char_params = mCharParams[0];
	const auto& kin_char = GetKinChar();
	const double rand_time = CalcRandKinResetTime();

	kin_char->Reset();
	kin_char->SetOriginRot(tQuaternion::Identity());
	kin_char->SetOriginPos(char_params.mInitPos); // reset origin
	kin_char->SetTime(rand_time);
	kin_char->Pose(rand_time);

	if (EnabledRandRotReset())
	{
		double rand_theta = mRand.RandDouble(-M_PI, M_PI);
		kin_char->RotateOrigin(cMathUtil::EulerToQuaternion(tVector(0, rand_theta, 0, 0)));
	}
}

double cCarlSceneTaskHeading::CalcRandKinResetTime()
{
	const auto& kin_char = GetKinChar();
	double dur = kin_char->GetMotionDuration();
	double rand_time = cMathUtil::RandDouble(0, dur);
	return rand_time;
}