#pragma once

#include "DrawCarlScene.h"
#include "CarlSceneKinChar.h"

class cDrawCarlSceneKinChar : virtual public cDrawCarlScene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawCarlSceneKinChar();
	virtual ~cDrawCarlSceneKinChar();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	virtual void Keyboard(unsigned char key, double device_x, double device_y);

	std::string GetName() const;

protected:
	std::shared_ptr<cCarlSceneKinChar> mScene;
	bool mEnableDrawGaitPattern;
	bool mEnableDrawCharacterBody;
	std::vector<tVector> mColors;
	std::vector<std::deque<bool>> mGaits;
	std::vector<int> mGaitJoints;

	virtual void BuildScene(std::shared_ptr<cCarlSceneKinChar>& out_scene) const;
	virtual void SetupScene(std::shared_ptr<cCarlSceneKinChar>& out_scene);
	virtual void UpdateScene(double time_elapsed);

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;
	virtual tVector GetDefaultCamFocus() const;

	virtual void DrawGround() const;
	virtual void DrawGround3D() const;
	virtual void DrawCharacters() const;
	virtual void DrawInfo() const;
	virtual void DrawInfoText() const;
	virtual void DrawInfoGaitPattern() const;
	virtual void DrawMisc() const;
	virtual void DrawEndEffectors() const;

	virtual void ToggleGaitPattern();
	virtual void ToggleDrawCharacterBody();

	virtual void InitGaits();
	virtual void UpdateGaits();
};
