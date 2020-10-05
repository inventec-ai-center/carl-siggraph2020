#pragma once

#include "DrawCarlRLScene.h"
#include "CarlSceneTaskImitate.h"
#include "scenes/DrawSceneSimChar.h"
#include "anim/KinCharacter.h"

class cDrawCarlSceneTaskImitate : virtual public cDrawCarlRLScene, virtual public cDrawSceneSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDrawCarlSceneTaskImitate();
	virtual ~cDrawCarlSceneTaskImitate();

	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);

	virtual void Init();
	virtual void Clear();
	virtual void Update(double timestep);
	virtual bool IsEpisodeEnd() const;
	virtual bool CheckValidEpisode() const;

	virtual void Keyboard(unsigned char key, double device_x, double device_y);
	virtual void DrawKinChar(bool enable);

	virtual std::string GetName() const;

protected:

	bool mDrawKinChar;
	bool mEnableRandomAction;
	bool mEnableSyncRootInUpdate;
	std::vector<std::deque<bool>> mKinGaits;
	std::vector<std::deque<bool>> mSimGaits;

	virtual cRLScene* GetRLScene() const;
	virtual cCarlRLScene* GetCarlRLScene() const;
	virtual const std::shared_ptr<cKinCharacter>& GetKinChar() const;

	virtual void BuildScene(std::shared_ptr<cSceneSimChar>& out_scene) const;
	virtual void DrawCharacters() const;
	virtual void DrawKinCharacters() const;
	virtual void DrawKinCharacter(const std::shared_ptr<cKinCharacter>& kin_char) const;
	virtual void DrawInfo() const;
	virtual void DrawInfoText() const;
	virtual void DrawGaitPatternInfo() const;

	virtual void ToggleRandomAction();
	virtual void ToggleSyncRootPosition();

	virtual void InitGaits();
	virtual void UpdateGait();
};
