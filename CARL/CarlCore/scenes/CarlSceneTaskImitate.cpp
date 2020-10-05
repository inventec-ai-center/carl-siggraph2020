#include "CarlSceneTaskImitate.h"
#include <math.h>
#include "sim/RBDUtil.h"
#include "sim/CtController.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "sim/CharController.h"
#include "render/DrawUtil.h"
#include "anim/CarlKinTree.h"

const int num_future_timesteps = 2;

// 0: use end-effector positional error
// 1: use contact state error
#define Use_Contact_Reward 1

// 0: use thresholding (the same as kinChar) as contact solver
// 1: use collision as contact solver
#define Use_Collision_Contact_Solver 0

double cCarlSceneTaskImitate::CalcRewardImitate(const cSimCharacter& sim_char, const cCarlKinCharacter& kin_char) const
{
#if Use_Contact_Reward
	double pose_w = 0.65;
	double vel_w = 0.1;
	double com_w = 0.1;
	double contact_w = 0.15;

	double total_w = pose_w + vel_w + contact_w + com_w;
	pose_w /= total_w;
	vel_w /= total_w;
	com_w /= total_w;
	contact_w /= total_w;

	const double pose_scale = 2;
	const double vel_scale = 0.1;
	const double com_scale = 10;
	const double contact_scale = 5;
	const double err_scale = 1;

	const auto& joint_mat = sim_char.GetJointMat();
	const auto& body_defs = sim_char.GetBodyDefs();
	double reward = 0;

	const Eigen::VectorXd& pose0 = sim_char.GetPose();
	const Eigen::VectorXd& vel0 = sim_char.GetVel();
	const Eigen::VectorXd& pose1 = kin_char.GetPose();
	const Eigen::VectorXd& vel1 = kin_char.GetVel();
	tMatrix origin_trans = sim_char.BuildOriginTrans();
	tMatrix kin_origin_trans = kin_char.BuildOriginTrans();

	tVector com0_world = sim_char.CalcCOM();
	tVector com_vel0_world = sim_char.CalcCOMVel();
	tVector com1_world;
	tVector com_vel1_world;
	cRBDUtil::CalcCoM(joint_mat, body_defs, pose1, vel1, com1_world, com_vel1_world);

	int root_id = sim_char.GetRootID();
	tVector root_pos0 = cCarlKinTree::GetRootPos(joint_mat, pose0);
	tVector root_pos1 = cCarlKinTree::GetRootPos(joint_mat, pose1);
	tQuaternion root_rot0 = cCarlKinTree::GetRootRot(joint_mat, pose0);
	tQuaternion root_rot1 = cCarlKinTree::GetRootRot(joint_mat, pose1);
	tVector root_vel0 = cCarlKinTree::GetRootVel(joint_mat, vel0);
	tVector root_vel1 = cCarlKinTree::GetRootVel(joint_mat, vel1);
	tVector root_ang_vel0 = cCarlKinTree::GetRootAngVel(joint_mat, vel0);
	tVector root_ang_vel1 = cCarlKinTree::GetRootAngVel(joint_mat, vel1);

	double pose_err = 0;
	double vel_err = 0;
	double com_err = 0;
	double contact_err = 0;

	int num_end_effs = 0;
	int num_joints = sim_char.GetNumJoints();
	assert(num_joints == mJointWeights.size());

	double root_rot_w = mJointWeights[root_id];
	pose_err += root_rot_w * cCarlKinTree::CalcRootRotErr(joint_mat, pose0, pose1);
	vel_err += root_rot_w * cCarlKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1);

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		double w = mJointWeights[j];
		double curr_pose_err = cCarlKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
		double curr_vel_err = cCarlKinTree::CalcVelErr(joint_mat, j, vel0, vel1);
		pose_err += w * curr_pose_err;
		vel_err += w * curr_vel_err;
	}

	int num_contacts = GetNumContacts();
	std::vector<bool> kin_contacts = GetKinCharContacts();
	std::vector<bool> sim_contacts = GetSimCharContacts(kin_char.GetJointMat());
	for (int i = 0; i < num_contacts; ++i)
	{
		if (sim_contacts[i] == true && kin_contacts[i] == false)
		{
			contact_err += 2;
		}
		else if (sim_contacts[i] == false && kin_contacts[i] == true)
		{
			contact_err += 1;
		}
	}

	if (num_contacts > 0)
	{
		contact_err /= num_contacts;
	}

	com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

	double pose_reward = exp(-err_scale * pose_scale * pose_err);
	double vel_reward = exp(-err_scale * vel_scale * vel_err);
	double com_reward = exp(-err_scale * com_scale * com_err);
	double contact_reward = exp(-err_scale * contact_scale * contact_err);

	reward = pose_w * pose_reward + vel_w * vel_reward + contact_w * contact_reward + com_w * com_reward;
	return reward;
#else
	double pose_w = 0.65;
	double vel_w = 0.1;
	double end_eff_w = 0.15;
	double com_w = 0.1;

	double total_w = pose_w + vel_w + end_eff_w + com_w;
	pose_w /= total_w;
	vel_w /= total_w;
	end_eff_w /= total_w;
	com_w /= total_w;

	const double pose_scale = 2;
	const double vel_scale = 0.1;
	const double end_eff_scale = 40;
	const double com_scale = 10;
	const double err_scale = 1;

	const auto& joint_mat = sim_char.GetJointMat();
	const auto& body_defs = sim_char.GetBodyDefs();
	double reward = 0;

	const Eigen::VectorXd& pose0 = sim_char.GetPose();
	const Eigen::VectorXd& vel0 = sim_char.GetVel();
	const Eigen::VectorXd& pose1 = kin_char.GetPose();
	const Eigen::VectorXd& vel1 = kin_char.GetVel();
	tMatrix origin_trans = sim_char.BuildOriginTrans();
	tMatrix kin_origin_trans = kin_char.BuildOriginTrans();

	tVector com0_world = sim_char.CalcCOM();
	tVector com_vel0_world = sim_char.CalcCOMVel();
	tVector com1_world;
	tVector com_vel1_world;
	cRBDUtil::CalcCoM(joint_mat, body_defs, pose1, vel1, com1_world, com_vel1_world);

	int root_id = sim_char.GetRootID();
	tVector root_pos0 = cCarlKinTree::GetRootPos(joint_mat, pose0);
	tVector root_pos1 = cCarlKinTree::GetRootPos(joint_mat, pose1);

	double pose_err = 0;
	double vel_err = 0;
	double end_eff_err = 0;
	double com_err = 0;
	double heading_err = 0;

	int num_end_effs = 0;
	int num_joints = sim_char.GetNumJoints();
	assert(num_joints == mJointWeights.size());

	double root_rot_w = mJointWeights[root_id];
	pose_err += root_rot_w * cCarlKinTree::CalcRootRotErr(joint_mat, pose0, pose1);
	vel_err += root_rot_w * cCarlKinTree::CalcRootAngVelErr(joint_mat, vel0, vel1);

	com_err = 0.1 * (com_vel1_world - com_vel0_world).squaredNorm();

	for (int j = root_id + 1; j < num_joints; ++j)
	{
		double w = mJointWeights[j];
		double curr_pose_err = cCarlKinTree::CalcPoseErr(joint_mat, j, pose0, pose1);
		double curr_vel_err = cCarlKinTree::CalcVelErr(joint_mat, j, vel0, vel1);
		pose_err += w * curr_pose_err;
		vel_err += w * curr_vel_err;

		bool is_end_eff = sim_char.IsEndEffector(j);
		if (is_end_eff)
		{
			tVector pos0 = sim_char.CalcJointPos(j);
			tVector pos1 = cCarlKinTree::CalcJointWorldPos(joint_mat, pose1, j);
			double ground_h0 = mGround->SampleHeight(pos0);
			double ground_h1 = kin_char.GetOriginPos()[1];

			tVector pos_rel0 = pos0 - root_pos0;
			tVector pos_rel1 = pos1 - root_pos1;
			pos_rel0[1] = pos0[1] - ground_h0;
			pos_rel1[1] = pos1[1] - ground_h1;

			pos_rel0 = origin_trans * pos_rel0;
			pos_rel1 = kin_origin_trans * pos_rel1;

			double curr_end_err = (pos_rel1 - pos_rel0).squaredNorm();
			end_eff_err += curr_end_err;
			++num_end_effs;
		}
	}

	if (num_end_effs > 0)
	{
		end_eff_err /= num_end_effs;
	}

	double pose_reward = exp(-err_scale * pose_scale * pose_err);
	double vel_reward = exp(-err_scale * vel_scale * vel_err);
	double end_eff_reward = exp(-err_scale * end_eff_scale * end_eff_err);
	double com_reward = exp(-err_scale * com_scale * com_err);

	reward = pose_w * pose_reward + vel_w * vel_reward + end_eff_w * end_eff_reward + com_w * com_reward;
	return reward;
#endif
}

cCarlSceneTaskImitate::cCarlSceneTaskImitate()
{
	mEnableRandRotReset = false;
	mSyncCharRootPos = true;
	mSyncCharRootRot = false;
	mEnableRootRotFail = false;
	mEnableRandomKinChar = true;
	mSyncCharRootInUpdate = false;
	mHoldEndFrame = 0;
	mTimestep = 0;
	mNextKinCharPhase = 0;
	mNextEndTime = 0;
}

cCarlSceneTaskImitate::~cCarlSceneTaskImitate()
{
}

void cCarlSceneTaskImitate::ParseArgs(const std::shared_ptr<cArgParser>& parser)
{
	cCarlRLSceneSimChar::ParseArgs(parser);
	parser->ParseString("motion_file", mMotionFile);
	parser->ParseBool("enable_rand_rot_reset", mEnableRandRotReset);
	parser->ParseBool("sync_char_root_pos", mSyncCharRootPos);
	parser->ParseBool("sync_char_root_rot", mSyncCharRootRot);
	parser->ParseBool("enable_root_rot_fail", mEnableRootRotFail);
	parser->ParseDouble("hold_end_frame", mHoldEndFrame);
}

void cCarlSceneTaskImitate::Init()
{
	mKinChar.reset();
	BuildKinChar();

	cCarlRLSceneSimChar::Init();
	InitJointWeights();
	InitGaits();
}

double cCarlSceneTaskImitate::CalcReward(int agent_id) const
{
	const cSimCharacter* sim_char = GetAgentChar(agent_id);
	bool fallen = HasFallen(*sim_char);

	double r = 0;
	int max_id = 0;
	if (!fallen)
	{
		r = CalcRewardImitate(*sim_char, *mKinChar);
	}
	return r;
}

const std::shared_ptr<cCarlKinCharacter>& cCarlSceneTaskImitate::GetKinChar() const
{
	return mKinChar;
}

void cCarlSceneTaskImitate::EnableRandRotReset(bool enable)
{
	mEnableRandRotReset = enable;
}

bool cCarlSceneTaskImitate::EnabledRandRotReset() const
{
	bool enable = mEnableRandRotReset;
	return enable;
}

void cCarlSceneTaskImitate::EnableRandomKinChar(bool enable)
{
	mEnableRandomKinChar = enable;
}

void cCarlSceneTaskImitate::EnableSyncKinCharRootInUpdate(bool enable)
{
	mSyncCharRootInUpdate = enable;
}

void cCarlSceneTaskImitate::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	const std::shared_ptr<cCarlKinCharacter> kin_char = GetKinChar();

	int kin_state_size = kin_char->GetStateSize();
	int goal_size = kin_state_size * num_future_timesteps;
	out_goal = Eigen::VectorXd(goal_size);
	out_goal.setZero();
	int next_count = 0;
	for (int i = 0; i < num_future_timesteps; ++i)
	{
		double next_time = kin_char->GetTime() + (i + 1) * mTimestep;
		Eigen::VectorXd next_state;
		if (next_time >= mNextEndTime)
		{
			next_time = mNextKinCharPhase + next_count * mTimestep;
			kin_char->RecordState(next_time, next_state);
			next_count++;
		}
		else
		{
			kin_char->RecordState(next_time, next_state);
		}
		out_goal.segment(i * kin_state_size, kin_state_size) = next_state;
	}
}

void cCarlSceneTaskImitate::RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const
{
	out_goal = Eigen::VectorXd(3);
	out_goal.setZero();

	tVector v_com = GetCharacter()->CalcCOMVel();
	out_goal(0) = v_com[0];
	out_goal(1) = v_com[2];
	out_goal(2) = v_com.norm();
}

int cCarlSceneTaskImitate::GetGoalSize(int agent_id) const
{
	return num_future_timesteps * GetKinChar()->GetStateSize();
}

void cCarlSceneTaskImitate::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int goal_size = GetGoalSize(agent_id);
	out_offset = Eigen::VectorXd::Zero(goal_size);
	out_scale = Eigen::VectorXd::Ones(goal_size);
}

void cCarlSceneTaskImitate::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	int goal_size = GetGoalSize(agent_id);
	out_groups = cCharController::gNormGroupSingle * Eigen::VectorXi::Ones(goal_size);
}

int cCarlSceneTaskImitate::GetNumActions(int agent_id) const
{
	return 1;
}

int cCarlSceneTaskImitate::GetNumContacts() const
{
	return mGaitJoints.size();
}

std::vector<bool> cCarlSceneTaskImitate::GetKinCharContacts() const
{
	const std::shared_ptr<cCarlKinCharacter> kin_char = GetKinChar();
	const Eigen::MatrixXd& joint_mat = kin_char->GetJointMat();
	const Eigen::MatrixXd& pose = kin_char->GetPose();

	std::vector<bool> contacts(mGaitJoints.size());
	for (int i = 0; i < mGaitJoints.size(); ++i)
	{
		int joint_id = mGaitJoints[i];
		double thres = cCarlKinTree::GetContactThreshold(joint_mat, joint_id);
		tVector offset = cCarlKinTree::GetContactOffset(joint_mat, joint_id);
		tMatrix m = cCarlKinTree::JointWorldTrans(joint_mat, pose, joint_id);
		tVector pos_contact = m * offset;

		bool contact = pos_contact[1] < thres;
		contacts[i] = contact;
	}
	return contacts;
}

std::vector<bool> cCarlSceneTaskImitate::GetSimCharContacts(const Eigen::MatrixXd& joint_mat) const
{
	const std::shared_ptr<cSimCharacter> sim_char = GetCharacter();

#if !Use_Collision_Contact_Solver
	const Eigen::MatrixXd& pose = sim_char->GetPose();
#endif

	std::vector<bool> contacts(mGaitJoints.size());
	for (int i = 0; i < mGaitJoints.size(); ++i)
	{
		int joint_id = mGaitJoints[i];
#if Use_Collision_Contact_Solver
		bool contact = sim_char->GetBodyPart(joint_id)->IsInContact();
		contacts[i] = contact;
#else
		double thres = cCarlKinTree::GetContactThreshold(joint_mat, joint_id);
		tVector offset = cCarlKinTree::GetContactOffset(joint_mat, joint_id);
		tMatrix m = cCarlKinTree::JointWorldTrans(joint_mat, pose, joint_id);
		tVector pos_contact = m * offset;
		bool contact = pos_contact[1] < thres;
		contacts[i] = contact;
#endif
	}

	return contacts;
}

cCarlSceneTaskImitate::eTerminate cCarlSceneTaskImitate::CheckTerminate(int agent_id) const
{
	eTerminate terminated = cCarlRLSceneSimChar::CheckTerminate(agent_id);
	if (terminated == eTerminateNull)
	{
		bool end_motion = false;
		const auto& kin_char = GetKinChar();
		const cMotion& motion = kin_char->GetMotion();

		if (motion.GetLoop() == cMotion::eLoopNone)
		{
			end_motion = false;
		}
		else
		{
			end_motion = kin_char->IsMotionOver();
		}

		terminated = (end_motion) ? eTerminateFail : terminated;
	}
	return terminated;
}

std::string cCarlSceneTaskImitate::GetName() const
{
	return "Task Imitation";
}

void cCarlSceneTaskImitate::RotateKinChar()
{
	const auto& kin_char = GetKinChar();
	kin_char->RotateOrigin(cMathUtil::EulerToQuaternion(tVector(0, 0.05f, 0, 0)));
	CalcReward(0);
}

bool cCarlSceneTaskImitate::BuildCharacters()
{
	bool succ = cCarlRLSceneSimChar::BuildCharacters();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
	return succ;
}

void cCarlSceneTaskImitate::CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const
{
	int num_joints = character->GetNumJoints();
	out_weights = Eigen::VectorXd::Ones(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		double curr_w = character->GetJointDiffWeight(j);
		out_weights[j] = curr_w;
	}

	double sum = out_weights.lpNorm<1>();
	out_weights /= sum;
}

bool cCarlSceneTaskImitate::BuildController(const cCtrlBuilder::tCtrlParams& ctrl_params, std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cSceneSimChar::BuildController(ctrl_params, out_ctrl);
	if (succ)
	{
		auto ct_ctrl = dynamic_cast<cCtController*>(out_ctrl.get());
		if (ct_ctrl != nullptr)
		{
			const auto& kin_char = GetKinChar();
			double cycle_dur = kin_char->GetMotionDuration();
			ct_ctrl->SetCyclePeriod(cycle_dur);
		}
	}
	return succ;
}

void cCarlSceneTaskImitate::BuildKinChar()
{
	bool succ = BuildKinCharacter(0, mMotionFile, mKinChar);
	if (!succ)
	{
		printf("Failed to build kin character\n");
		assert(false);
	}
}

bool cCarlSceneTaskImitate::BuildKinCharacter(int id, std::string motion_file, std::shared_ptr<cCarlKinCharacter>& out_char) const
{
	auto kin_char = std::shared_ptr<cCarlKinCharacter>(new cCarlKinCharacter());
	const cSimCharacter::tParams& sim_char_params = mCharParams[0];
	cCarlKinCharacter::tParams kin_char_params;

	kin_char_params.mID = id;
	kin_char_params.mCharFile = sim_char_params.mCharFile;
	kin_char_params.mOrigin = sim_char_params.mInitPos;
	kin_char_params.mLoadDrawShapes = true;
	kin_char_params.mMotionFile = motion_file;

	bool succ = kin_char->Init(kin_char_params);
	if (succ)
	{
		out_char = kin_char;
	}
	return succ;
}

void cCarlSceneTaskImitate::UpdateCharacters(double timestep)
{
	UpdateKinChar(timestep);
	cCarlRLSceneSimChar::UpdateCharacters(timestep);
}

void cCarlSceneTaskImitate::UpdateKinChar(double timestep)
{
	auto kin_char = GetKinChar();
	kin_char->Update(timestep);
	double curr_time = kin_char->GetTime();

	if (curr_time >= mNextEndTime)
	{
		UpdateNextRandKinChar(false);

		kin_char = GetKinChar();
		kin_char->SetTime(mNextKinCharPhase);
		kin_char->Pose(mNextKinCharPhase);

		const auto& sim_char = GetCharacter();
		SyncKinCharNewCycle(*sim_char, *kin_char);
	}

	if (mSyncCharRootInUpdate)
	{
		SyncKinCharRoot();
	}
}

void cCarlSceneTaskImitate::ResetCharacters()
{
	cCarlRLSceneSimChar::ResetCharacters();

	ResetKinChar();
	if (EnableSyncChar())
	{
		SyncCharacters();
	}
}

void cCarlSceneTaskImitate::ResetKinChar()
{
	UpdateNextRandKinChar(true);

	const cSimCharacter::tParams& char_params = mCharParams[0];
	const auto& kin_char = GetKinChar();

	kin_char->Reset();
	kin_char->SetOriginRot(tQuaternion::Identity());
	kin_char->SetOriginPos(char_params.mInitPos); // reset origin
	kin_char->SetTime(mNextKinCharPhase);
	kin_char->Pose(mNextKinCharPhase);

	if (EnabledRandRotReset())
	{
		double rand_theta = mRand.RandDouble(-M_PI, M_PI);
		kin_char->RotateOrigin(cMathUtil::EulerToQuaternion(tVector(0, rand_theta, 0, 0)));
	}
}

void cCarlSceneTaskImitate::SyncCharacters()
{
	const auto& kin_char = GetKinChar();
	const Eigen::VectorXd& pose = kin_char->GetPose();
	const Eigen::VectorXd& vel = kin_char->GetVel();

	const auto& sim_char = GetCharacter();
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

bool cCarlSceneTaskImitate::EnableSyncChar() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->HasMotion();
}

void cCarlSceneTaskImitate::InitCharacterPosFixed(const std::shared_ptr<cSimCharacter>& out_char)
{

}

void cCarlSceneTaskImitate::InitGaits()
{
	mGaitJoints.clear();

	const std::shared_ptr<cCarlKinCharacter> kin_char = GetKinChar();
	const Eigen::MatrixXd& joint_mat = kin_char->GetJointMat();
	for (int i = 0; i < kin_char->GetNumJoints(); ++i)
	{
		if (cCarlKinTree::IsFootContact(joint_mat, i))
		{
			mGaitJoints.push_back(i);
		}
	}
}

void cCarlSceneTaskImitate::InitJointWeights()
{
	CalcJointWeights(GetCharacter(), mJointWeights);
}

void cCarlSceneTaskImitate::ResolveCharGroundIntersect()
{
	cCarlRLSceneSimChar::ResolveCharGroundIntersect();

	if (EnableSyncChar())
	{
		SyncKinCharRoot();
	}
}

void cCarlSceneTaskImitate::ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const
{
	cCarlRLSceneSimChar::ResolveCharGroundIntersect(out_char);
}

void cCarlSceneTaskImitate::SyncKinCharRoot()
{
	const auto& sim_char = GetCharacter();
	tVector sim_root_pos = sim_char->GetRootPos();
	double sim_heading = sim_char->CalcHeading();

	const auto& kin_char = GetKinChar();
	double kin_heading = kin_char->CalcHeading();

	tQuaternion drot = tQuaternion::Identity();
	if (mSyncCharRootRot)
	{
		drot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), sim_heading - kin_heading);
	}

	kin_char->RotateRoot(drot);
	kin_char->SetRootPos(sim_root_pos);
}

void cCarlSceneTaskImitate::SyncKinCharNewCycle(const cSimCharacter& sim_char, cCarlKinCharacter& out_kin_char) const
{
	if (mSyncCharRootRot)
	{
		double sim_heading = sim_char.CalcHeading();
		double kin_heading = out_kin_char.CalcHeading();
		tQuaternion drot = cMathUtil::AxisAngleToQuaternion(tVector(0, 1, 0, 0), sim_heading - kin_heading);
		out_kin_char.RotateRoot(drot);
	}

	if (mSyncCharRootPos)
	{
		tVector sim_root_pos = sim_char.GetRootPos();
		tVector kin_root_pos = out_kin_char.GetRootPos();
		kin_root_pos[0] = sim_root_pos[0];
		kin_root_pos[2] = sim_root_pos[2];

		tVector origin = out_kin_char.GetOriginPos();
		double dh = kin_root_pos[1] - origin[1];
		double ground_h = mGround->SampleHeight(kin_root_pos);
		kin_root_pos[1] = ground_h + dh;

		out_kin_char.SetRootPos(kin_root_pos);
	}
}

double cCarlSceneTaskImitate::GetKinTime() const
{
	const auto& kin_char = GetKinChar();
	return kin_char->GetTime();
}

bool cCarlSceneTaskImitate::CheckKinNewCycle(double timestep) const
{
	bool new_cycle = false;
	const auto& kin_char = GetKinChar();
	if (kin_char->GetMotion().EnableLoop())
	{
		double cycle_dur = kin_char->GetMotionDuration();
		double time = GetKinTime();
		new_cycle = cMathUtil::CheckNextInterval(timestep, time, cycle_dur);
	}
	return new_cycle;
}

bool cCarlSceneTaskImitate::HasFallen(const cSimCharacter& sim_char) const
{
	bool fallen = cCarlRLSceneSimChar::HasFallen(sim_char);
	if (mEnableRootRotFail)
	{
		fallen |= CheckRootRotFail(sim_char);
	}

	return fallen;
}

bool cCarlSceneTaskImitate::CheckRootRotFail(const cSimCharacter& sim_char) const
{
	const auto& kin_char = GetKinChar();
	bool fail = CheckRootRotFail(sim_char, *kin_char);
	return fail;
}

bool cCarlSceneTaskImitate::CheckRootRotFail(const cSimCharacter& sim_char, const cCarlKinCharacter& kin_char) const
{
	const double threshold = 0.5 * M_PI;

	tQuaternion sim_rot = sim_char.GetRootRotation();
	tQuaternion kin_rot = kin_char.GetRootRotation();
	double rot_diff = cMathUtil::QuatDiffTheta(sim_rot, kin_rot);
	return rot_diff > threshold;
}

void cCarlSceneTaskImitate::UpdateNextRandKinChar(bool rand_phase)
{
	if (!mEnableRandomKinChar || !rand_phase)
	{
		mNextEndTime = GetKinChar()->GetMotionDuration() - num_future_timesteps * mTimestep;
		mNextKinCharPhase = 0;
	}
	else
	{
		mNextEndTime = GetKinChar()->GetMotionDuration() - num_future_timesteps * mTimestep;
		mNextKinCharPhase = cMathUtil::RandDouble(0, mNextEndTime - num_future_timesteps * mTimestep);
	}
}
