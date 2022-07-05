#pragma once
#include "model_tools.h"

class Drone
{
public:
	double viewDis;
	double safeDis;
	double vFov;
	double hFov;
	double vOverlap;
	double hOverlap;

	Drone() = delete;
	Drone(const double&, const double&, const double&, const double&, const double&, const double&);
};