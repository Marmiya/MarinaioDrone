#pragma once
#include "model_tools.h"

enum stateOfCell
{
	unknown, occupied, dangerous, safe
};
class Perception
{
public:
	PointSet3 pts;
	std::tuple<double, double, double, double> range;
	std::vector<std::vector<stateOfCell>> stateNet;
	double xGridIndexA;
	double yGridIndexA;
	double CCL;
	Perception() = default;
	Perception(const std::tuple<double, double, double, double>& range,
		const std::vector<double>& xgp, const std::vector<double>& ygp);
	void initialization(const Point3& initialPos);
	bool Pcptcheck() const;
};