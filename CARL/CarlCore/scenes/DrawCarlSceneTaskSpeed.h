#pragma once

#include "DrawCarlRLScene.h"
#include "CarlSceneTaskSpeed.h"
#include "scenes/DrawSceneSimChar.h"

class cDrawCarlSceneTaskSpeed : virtual public cDrawCarlRLScene, virtual public cDrawSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawCarlSceneTaskSpeed();
	virtual ~cDrawCarlSceneTaskSpeed();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void Init();
	virtual void Clear();
	virtual bool IsEpisodeEnd() const;
	virtual bool CheckValidEpisode() const;

	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, double device_x, double device_y);

	virtual std::string GetName() const;

protected:

	enum CtrlMode
	{
		eManual,
		eRandom,
		eMaxMode
	};

	CtrlMode mCtrlMode;
	bool mEnableRandomVelocity;

	virtual cRLScene* GetRLScene() const;
	virtual cCarlRLScene* GetCarlRLScene() const;
	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;

	virtual void ToggleCtrlMode();
	virtual void AddTargetVelocity(double delta);

	virtual void DrawInfo() const;
	virtual void DrawInfoText() const;
	virtual void DrawMisc() const;
	virtual void DrawCharacters() const;
	virtual void DrawHeading() const;
};
