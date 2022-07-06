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
		const std::tuple<double, double, double, double>& ,const double&);
	void whatWeLook(const RTCScene&);

private:
	bool SASGen(const double&);
};