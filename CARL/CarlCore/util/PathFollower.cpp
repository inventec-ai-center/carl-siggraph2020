#include "PathFollower.h"
#include "render/DrawUtil.h"
#include "util/MathUtilExtend.h"
#include <fstream>

const int gNumLineSegments = 1000;
const double gAgentDistance = 1.0;
const double gProgressSpeed = 0.01;
const tVector gLineColor = tVector(0, 0, 0, 0.5);
const tVector gCtrlPtColor = tVector(0, 1, 0, 0.5);
const tVector gProgressPtColor = tVector(1, 0, 0, 0.5);

cPathFollower::cPathFollower()
{
	mProgress = 0;
}

cPathFollower::~cPathFollower()
{
}

void cPathFollower::Init(std::vector<tVector> ctrl_pts)
{
	mPath = std::shared_ptr<PathApproximator>(new PathApproximator());
	mPath->AddPoints(ctrl_pts);
}

void cPathFollower::Update(tVector position)
{
	double diff = (GetProgressPoint() - position).norm();
	mProgress += (diff < gAgentDistance) ? gProgressSpeed : 0;
	if (mProgress >= 0.999)
	{
		mProgress = 0.999;
	}
}

void cPathFollower::Reset()
{
	mProgress = 0.05;
}

void cPathFollower::Clear()
{

}

tVector cPathFollower::GetProgressPoint() const
{
	return mPath->GetPoint(mProgress);
}

double cPathFollower::GetResetHeading() const
{
	tVector p0 = mPath->GetPoint(0);
	tVector p1 = mPath->GetPoint(gProgressSpeed);
	tVector heading_dir = (p1 - p0).normalized();
	double heading = cMathUtilExtend::ClampEuler(atan2(-heading_dir[2], heading_dir[0]));
	return heading;
}

void cPathFollower::Draw() const
{
	DrawPath();
	DrawProgress();
}

void cPathFollower::DrawPath() const
{
	double step = 1.0 / (gNumLineSegments-1);
	std::vector<tVector> points = mPath->GetPoints();

	cDrawUtil::SetLineWidth(3.0f);
	cDrawUtil::SetColor(gLineColor);
	for (int i = 1; i < points.size(); ++i)
	{
		tVector a = points[i - 1];
		tVector b = points[i];
		a[1] += 0.01;
		b[1] += 0.01;
		cDrawUtil::DrawLine(a, b);
	}
}

void cPathFollower::DrawProgress() const
{
	tVector p = mPath->GetPoint(mProgress);
	cDrawUtil::PushMatrixView();
	cDrawUtil::SetColor(gProgressPtColor);
	cDrawUtil::Translate(p);
	cDrawUtil::DrawSphere(0.05f);
	cDrawUtil::PopMatrixView();
}

std::vector<tVector> cPathFollower::ParsePathCtrlPoints(std::string filename)
{
	std::vector<tVector> points;
	std::ifstream fin(filename);
	if (fin.is_open())
	{
		tVector p = tVector::Zero();
		while (fin >> p[0] >> p[1] >> p[2])
		{
			points.push_back(p);
		}
		fin.close();
	}
	else
	{
		printf("Path point file %s not found\n");
		assert(0);
	}
	return points;
}
