#pragma once

#include "scenes/Scene.h"
#include "anim/CarlKinCharacter.h"

class cCarlSceneKinChar: virtual public cScene
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cCarlSceneKinChar();
	virtual ~cCarlSceneKinChar();

	virtual void Init();
	virtual void ParseArgs(const std::shared_ptr<cArgParser>& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual const std::shared_ptr<cCarlKinCharacter>& GetCharacter() const;
	virtual tVector GetCharPos() const;
	virtual double GetTime() const;

	virtual std::string GetName() const;

protected:
	cCarlKinCharacter::tParams mCharParams;
	std::shared_ptr<cCarlKinCharacter> mChar;

	virtual void ParseCharParams(const std::shared_ptr<cArgParser>& parser, cCarlKinCharacter::tParams& out_params) const;

	virtual bool BuildCharacters();
	virtual bool BuildCharacter(const cCarlKinCharacter::tParams& params, std::shared_ptr<cCarlKinCharacter>& out_char) const;
	virtual void ResetCharacters();
	virtual void UpdateCharacters(double timestep);
};
