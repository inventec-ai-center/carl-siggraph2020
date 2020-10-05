#include "DeepMimicCore.h"
#include "scenes/CarlRLScene.h"


class cCarlCore: public cDeepMimicCore
{
public:
	cCarlCore(bool enable_draw);
	virtual ~cCarlCore();

    virtual void Init();
    virtual void Reshape(int w, int h);
    virtual bool IsCarlRLScene() const;
    virtual std::vector<double> RecordGoalTarget(int agent_id) const;
    virtual void LogGatingWeights(int agent_id, const std::vector<double>& weights);
	virtual void LogPrimitivesMeanStd(int agent_id, int num_primitives, const std::vector<double>& means, const std::vector<double>& stds);

protected:

    std::shared_ptr<cCarlRLScene> mCarlRLScene;

    virtual void SetupScene();
    virtual const std::shared_ptr<cCarlRLScene>& GetCarlRLScene() const;
};
