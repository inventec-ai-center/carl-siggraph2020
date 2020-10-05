#pragma once

#include "CarlRLSceneSimChar.h"
#include "anim/CarlKinCharacter.h"

class cCarlSceneTaskImitate : virtual public cCarlRLSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCarlSceneTaskImitate();
	virtual ~cCarlSceneTaskImitate();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual const std::shared_ptr<cCarlKinCharacter>& GetKinChar() const;
	virtual void EnableRandRotReset(bool enable);
	virtual bool EnabledRandRotReset() const;

	virtual void EnableRandomKinChar(bool enable);
	virtual void EnableSyncKinCharRootInUpdate(bool enable);

	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual void RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const;
	int GetGoalSize(int agent_id) const;
	void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;
	int GetNumActions(int agent_id) const;

	int GetNumContacts() const;
	std::vector<bool> GetKinCharContacts() const;
	std::vector<bool> GetSimCharContacts(const Eigen::MatrixXd& joint_mat) const;

	virtual double CalcReward(int agent_id) const;
	virtual eTerminate CheckTerminate(int agent_id) const;

	virtual std::string GetName() const;

	virtual void RotateKinChar();

protected:

	std::string mMotionFile;
	std::shared_ptr<cCarlKinCharacter> mKinChar;

	Eigen::VectorXd mJointWeights;
	bool mEnableRandRotReset;
	bool mSyncCharRootPos;
	bool mSyncCharRootRot;
	bool mEnableRootRotFail;
	bool mEnableRandomKinChar;
	bool mSyncCharRootInUpdate;
	double mHoldEndFrame;
	double mTimestep;
	double mNextKinCharPhase;
	double mNextEndTime;

	std::vector<int> mGaitJoints;

	virtual bool BuildCharacters();

	virtual void CalcJointWeights(const std::shared_ptr<cSimCharacter>& character, Eigen::VectorXd& out_weights) const;
	virtual bool BuildController(const cCtrlBuilder::tCtrlParams& ctrl_params, std::shared_ptr<cCharController>& out_ctrl);
	virtual void BuildKinChar();
	virtual bool BuildKinCharacter(int id, std::string motion_file, std::shared_ptr<cCarlKinCharacter>& out_char) const;
	virtual void UpdateCharacters(double timestep);
	virtual void UpdateKinChar(double timestep);

	virtual void ResetCharacters();
	virtual void ResetKinChar();
	virtual void SyncCharacters();
	virtual bool EnableSyncChar() const;
	virtual void InitCharacterPosFixed(const std::shared_ptr<cSimCharacter>& out_char);

	virtual void InitGaits();
	virtual void InitJointWeights();
	virtual void ResolveCharGroundIntersect();
	virtual void ResolveCharGroundIntersect(const std::shared_ptr<cSimCharacter>& out_char) const;
	virtual void SyncKinCharRoot();
	virtual void SyncKinCharNewCycle(const cSimCharacter& sim_char, cCarlKinCharacter& out_kin_char) const;

	virtual double GetKinTime() const;
	virtual bool CheckKinNewCycle(double timestep) const;
	virtual bool HasFallen(const cSimCharacter& sim_char) const;
	virtual bool CheckRootRotFail(const cSimCharacter& sim_char) const;
	virtual bool CheckRootRotFail(const cSimCharacter& sim_char, const cCarlKinCharacter& kin_char) const;
	
	virtual void UpdateNextRandKinChar(bool rand_phase);

	virtual double CalcRewardImitate(const cSimCharacter& sim_char, const cCarlKinCharacter& ref_char) const;
};