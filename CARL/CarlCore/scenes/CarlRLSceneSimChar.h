#pragma once

#include "scenes/RLSceneSimChar.h"
#include "CarlRLScene.h"

class cCarlRLSceneSimChar: virtual public cRLSceneSimChar, virtual public cCarlRLScene
{
public:

    virtual void RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const;
    virtual void LogGatingWeights(int agent_id, const std::vector<double>& weights);
	virtual void LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds);
    virtual double CalcReward(int agent_id) const;

protected:


};
