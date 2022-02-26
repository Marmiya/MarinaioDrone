#pragma once
#include "viewpoint.h"
#include <numeric>

double
fundenmentalREC(
	const std::vector<Viewpoint>& vs, const Viewpoint& v,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal
);

double
totalREC(
	const std::vector<Viewpoint>& trajectory, 
	const std::vector<Eigen::Vector3d>& ptsp, const std::vector<Eigen::Vector3d>& ptsn,
	std::vector<std::vector<int>>& visibility
);

std::vector<double>
totalRECv(
	const std::vector<Viewpoint>& trajectory,
	const std::vector<Eigen::Vector3d>& ptsp, const std::vector<Eigen::Vector3d>& ptsn,
	std::vector<std::vector<int>>& visibility
);