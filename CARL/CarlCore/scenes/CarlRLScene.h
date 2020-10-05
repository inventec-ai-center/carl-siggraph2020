#pragma once

#include "scenes/RLScene.h"

class cCarlRLScene : virtual public cRLScene
{
public:
    virtual void RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const = 0;
    virtual void LogGatingWeights(int agent_id, const std::vector<double>& weights);
	virtual void LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds);

protected:

};
