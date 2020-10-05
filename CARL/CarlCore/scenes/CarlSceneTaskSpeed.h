#pragma once

#include "CarlRLSceneSimChar.h"
#include "anim/KinCharacter.h"

class cCarlSceneTaskSpeed : virtual public cCarlRLSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCarlSceneTaskSpeed();
	virtual ~cCarlSceneTaskSpeed();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Init();

	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;
	virtual void EnableRandRotReset(bool enable);
	virtual bool EnabledRandRotReset() const;

	virtual void EnableRandVelocity(bool enable);
	virtual void SetTargetVelocity(double velocity);
	virtual double GetTargetVelocity() const;
	virtual double GetTargetHeading() const;

	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual void RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const;
	int GetGoalSize(int agent_id) const;
	void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual double CalcReward(int agent_id) const;

	virtual std::string GetName() const;

protected:

	typedef struct {
		double fromTime;
		double toTime;
		double fromSpeed;
		double toSpeed;
	} SpeedCtrlInterval;

	std::string mMotionFile;
	std::shared_ptr<cKinCharacter> mKinChar;

	bool mEnableRandRotReset;
	bool mEnableRandVelocity;
	bool mEnableRootRotFail;
	double mTargetVelocity;
	double mTargetHeading;
	double mMinRandVelocity;
	double mMaxRandVelocity;
	int mUpdateCount;

	virtual bool BuildCharacters();
	virtual void BuildKinChars();
	virtual bool BuildKinCharacter(int id, std::shared_ptr<cKinCharacter>& out_char) const;

	virtual void UpdateCharacters(double timestep);
	virtual void UpdateTargetVelocity(double timestep);

	virtual double GetKinTime() const;
	virtual void SyncCharacters();
	virtual bool EnableSyncChar() const;
	virtual void ResetCharacters();
	virtual void ResetKinChar();

	virtual double CalcRandKinResetTime();
	virtual double CalcRewardVelocity(const cSimCharacter& sim_char, const double target_velocity) const;
};