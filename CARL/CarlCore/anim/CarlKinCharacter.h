#pragma once

#include "anim/KinCharacter.h"

class cCarlKinCharacter : virtual public cKinCharacter
{
public:

    virtual bool LoadSkeleton(const Json::Value& root);
	virtual int GetStateSize() const;
	virtual void RecordState(double time, Eigen::VectorXd& out_state) const;

protected:


};
