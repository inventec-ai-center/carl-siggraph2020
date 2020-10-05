#pragma once

#include <string>
#include "util/MathUtil.h"

class PathApproximator
{
public:
	PathApproximator();
	~PathApproximator();

	void AddPoints(const std::vector<tVector>& points);
	tVector GetPoint(double t) const;
	const std::vector<tVector>& GetPoints() const;
	double GetTotalDistance() const;

private:
	std::vector<tVector> mPoints;
	std::vector<double> mIntegralDistances;

	void BuildIntegralDistances();
};
