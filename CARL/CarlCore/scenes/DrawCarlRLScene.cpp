#include "DrawCarlRLScene.h"

cDrawCarlRLScene::cDrawCarlRLScene()
{
}

cDrawCarlRLScene::~cDrawCarlRLScene()
{
}

void cDrawCarlRLScene::Init()
{
	cCarlRLScene::Init();
	cDrawCarlScene::Init();
}

int cDrawCarlRLScene::GetNumAgents() const
{
	return GetCarlRLScene()->GetNumAgents();
}

bool cDrawCarlRLScene::NeedNewAction(int agent_id) const
{
	return GetCarlRLScene()->NeedNewAction(agent_id);
}

void cDrawCarlRLScene::RecordState(int agent_id, Eigen::VectorXd& out_state) const
{
	GetCarlRLScene()->RecordState(agent_id, out_state);
}

void cDrawCarlRLScene::RecordGoal(int agent_id, Eigen::VectorXd& out_goal) const
{
	GetCarlRLScene()->RecordGoal(agent_id, out_goal);
}

void cDrawCarlRLScene::RecordGoalTarget(int agent_id, Eigen::VectorXd& out_goal) const
{
	GetCarlRLScene()->RecordGoalTarget(agent_id, out_goal);
}

void cDrawCarlRLScene::SetAction(int agent_id, const Eigen::VectorXd& action)
{
	GetCarlRLScene()->SetAction(agent_id, action);
}

eActionSpace cDrawCarlRLScene::GetActionSpace(int agent_id) const
{
	return GetCarlRLScene()->GetActionSpace(agent_id);
}

int cDrawCarlRLScene::GetStateSize(int agent_id) const
{
	return GetCarlRLScene()->GetStateSize(agent_id);
}

int cDrawCarlRLScene::GetGoalSize(int agent_id) const
{
	return GetCarlRLScene()->GetGoalSize(agent_id);
}

int cDrawCarlRLScene::GetActionSize(int agent_id) const
{
	return GetCarlRLScene()->GetActionSize(agent_id);
}

int cDrawCarlRLScene::GetNumActions(int agent_id) const
{
	return GetCarlRLScene()->GetNumActions(agent_id);
}

void cDrawCarlRLScene::BuildStateOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetCarlRLScene()->BuildStateOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawCarlRLScene::BuildGoalOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetCarlRLScene()->BuildGoalOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawCarlRLScene::BuildActionOffsetScale(int agent_id, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	GetCarlRLScene()->BuildActionOffsetScale(agent_id, out_offset, out_scale);
}

void cDrawCarlRLScene::BuildActionBounds(int agent_id, Eigen::VectorXd& out_min, Eigen::VectorXd& out_max) const
{
	GetCarlRLScene()->BuildActionBounds(agent_id, out_min, out_max);
}

void cDrawCarlRLScene::BuildStateNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	GetCarlRLScene()->BuildStateNormGroups(agent_id, out_groups);
}

void cDrawCarlRLScene::BuildGoalNormGroups(int agent_id, Eigen::VectorXi& out_groups) const
{
	GetCarlRLScene()->BuildGoalNormGroups(agent_id, out_groups);
}

double cDrawCarlRLScene::CalcReward(int agent_id) const
{
	return GetCarlRLScene()->CalcReward(agent_id);
}

double cDrawCarlRLScene::GetRewardMin(int agent_id) const
{
	return GetCarlRLScene()->GetRewardMin(agent_id);
}

double cDrawCarlRLScene::GetRewardMax(int agent_id) const
{
	return GetCarlRLScene()->GetRewardMax(agent_id);
}

double cDrawCarlRLScene::GetRewardFail(int agent_id)
{
	return GetCarlRLScene()->GetRewardFail(agent_id);
}

double cDrawCarlRLScene::GetRewardSucc(int agent_id)
{
	return GetCarlRLScene()->GetRewardSucc(agent_id);
}

bool cDrawCarlRLScene::IsEpisodeEnd() const
{
	return GetCarlRLScene()->IsEpisodeEnd();
}

cDrawCarlRLScene::eTerminate cDrawCarlRLScene::CheckTerminate(int agent_id) const
{
	return GetCarlRLScene()->CheckTerminate(agent_id);
}

bool cDrawCarlRLScene::CheckValidEpisode() const
{
	return GetCarlRLScene()->CheckValidEpisode();
}

void cDrawCarlRLScene::SetMode(eMode mode)
{
	return GetCarlRLScene()->SetMode(mode);
}

void cDrawCarlRLScene::SetSampleCount(int count)
{
	return GetCarlRLScene()->SetSampleCount(count);
}

void cDrawCarlRLScene::LogVal(int agent_id, double val)
{
	GetCarlRLScene()->LogVal(agent_id, val);
}

void cDrawCarlRLScene::LogGatingWeights(int agent_id, const std::vector<double>& weights)
{
	GetCarlRLScene()->LogGatingWeights(agent_id, weights);
}

void cDrawCarlRLScene::LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds)
{
	GetCarlRLScene()->LogPrimitivesMeanStd(agent_id, num_primitives, means, stds);
}

std::string cDrawCarlRLScene::GetName() const
{
	return GetCarlRLScene()->GetName();
}

void cDrawCarlRLScene::BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const
{
	out_scene = std::shared_ptr<cCarlRLSceneSimChar>(new cCarlRLSceneSimChar());
}
