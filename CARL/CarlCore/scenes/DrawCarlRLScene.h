#pragma once

#include "DrawCarlScene.h"
#include "CarlRLScene.h"
#include "CarlRLSceneSimChar.h"
#include "scenes/SceneSimChar.h"

class cDrawCarlRLScene: virtual public cCarlRLScene, virtual public cDrawCarlScene
{
public:
	virtual ~cDrawCarlRLScene();

	virtual void Init();

	virtual int GetNumAgents() const;
	virtual bool NeedNewAction(int agent_id) const;
	virtual void RecordState(int agent_id, Eigen::VectorXd& out_state) const;
	virtual void RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual void RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const;
	virtual void SetAction(int agent_id, const Eigen::VectorXd& action);

	virtual eActionSpace GetActionSpace(int agent_id) const;
	virtual int GetStateSize(int agent_id) const;
	virtual int GetGoalSize(int agent_id) const;
	virtual int GetActionSize(int agent_id) const;
	virtual int GetNumActions(int agent_id) const;

	virtual void BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const;

	virtual void BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;
	virtual void BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const;

	virtual double CalcReward(int agent_id) const;
	virtual double GetRewardMin(int agent_id) const;
	virtual double GetRewardMax(int agent_id) const;
	virtual double GetRewardFail(int agent_id);
	virtual double GetRewardSucc(int agent_id);

	virtual bool IsEpisodeEnd() const;
	virtual eTerminate CheckTerminate(int agent_id) const;
	virtual bool CheckValidEpisode() const;
	virtual void SetMode(eMode mode);

	virtual void SetSampleCount(int count);
	virtual void LogVal(int agent_id, double val);
	virtual void LogGatingWeights(int agent_id, const std::vector<double>& weights);
	virtual void LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds);

	virtual std::string GetName() const;

protected:

	cDrawCarlRLScene();
	virtual cRLScene* GetRLScene() const = 0;
    virtual cCarlRLScene* GetCarlRLScene() const = 0;
    virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

    std::shared_ptr<cCarlRLSceneSimChar> mCarlScene;
};