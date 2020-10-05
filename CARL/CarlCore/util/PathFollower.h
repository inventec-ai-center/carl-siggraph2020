#pragma once

#include "sim/SimObj.h"
#include "util/PathApproximator.h"

class cPathFollower
{
public:
	cPathFollower();
	~cPathFollower();

	virtual void Init(std::vector<tVector> ctrl_pts);
	virtual void Update(tVector position);
	virtual void Reset();
	virtual void Clear();

	virtual tVector GetProgressPoint() const;
	virtual double GetResetHeading() const;

	virtual void Draw() const;

	static std::vector<tVector> ParsePathCtrlPoints(std::string filename);

protected:

	double mProgress;

	virtual void DrawPath() const;
	virtual void DrawProgress() const;

	std::shared_ptr<PathApproximator> mPath;
};

