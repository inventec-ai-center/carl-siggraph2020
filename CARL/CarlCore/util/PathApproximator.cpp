#include "PathApproximator.h"

int BinarySearch(const std::vector<double>& array, double value)
{
	auto it = upper_bound(array.begin(), array.end(), value, [](double a, double b){ return a < b; });
    int index = std::distance(array.begin(), it);
    return index;
}

PathApproximator::PathApproximator()
{
} 

PathApproximator::~PathApproximator()
{
}

void PathApproximator::AddPoints(const std::vector<tVector>& points)
{
	mPoints.resize(points.size());
	for (int i = 0; i < points.size(); ++i)
	{
		mPoints[i] = points[i];
	}

	BuildIntegralDistances();
}

void PathApproximator::BuildIntegralDistances()
{
	mIntegralDistances.clear();
	mIntegralDistances.resize(mPoints.size(), 0);
	for (int i = 1; i < mPoints.size(); ++i)
	{
		double distance = (mPoints[i] - mPoints[i - 1]).norm();
		mIntegralDistances[i] = distance + mIntegralDistances[i - 1];
	}
}

tVector PathApproximator::GetPoint(double t) const
{
	assert(t >= 0 && t <= 1);
	float target_distance = GetTotalDistance() * t;
	int closest_index = BinarySearch(mIntegralDistances, target_distance);
	tVector point = mPoints[closest_index];
	return point;
}

const std::vector<tVector>& PathApproximator::GetPoints() const
{
	return mPoints;
}

double PathApproximator::GetTotalDistance() const
{
	return mIntegralDistances[mIntegralDistances.size() - 1];
}