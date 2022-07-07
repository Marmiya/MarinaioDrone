#pragma once
#include "model_tools.h"
#include "Perception.h"
#include "intersection_tools.h"

class Drone
{
public:
	double viewDis;
	double safeDis;
	double vFov;
	double hFov;
	double vOverlap;
	double hOverlap;
	Eigen::Vector3d camBase{ 0., 0., -1.};
	std::vector<std::pair<double, double>> sightAngleSet;
	Point3 position;
	Vector3 direction;
	Perception perception;

	Drone() = delete;
	Drone(const double&, const double&, const double&, const double&, const double&, const double&);

	void initialization(const double& initx, const double& inity, const double& initz,
		const std::tuple<double, double, double, double>&,
		const std::vector<double>&, const std::vector<double>&,
		const double&);
	void updateUncertainty(const PointSet3& pts);
	void whatWeLook(const RTCScene&);
	void letsThink();
	void whatWeDo();

private:
	bool SASGen(const double&);
};