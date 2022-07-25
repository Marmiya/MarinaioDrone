#pragma once
#include "common_util.h"
#include "model_tools.h"
#include "metrics.h"
#include <cstdlib>
#include <random>
#include <unordered_set>
#include <corecrt_math_defines.h>
#include <CGAL/point_generators_3.h>
#include "viewpoint.h"
#include "CalRec.cuh"

#include "DroneScanViz.h"


extern std::array<std::pair<double, double>, 128> viewCandidates;


std::vector<Viewpoint> droneScan(
	PointSet3 &points, const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
	const int maxIterTimes, const int initViewNum, const std::string logPath,
	const double viewDis, const double maxAngle, const double maxDis, DroneScanViz* v_viz = nullptr,
	const bool& modl = true
);

std::vector<Viewpoint> droneScanAdj(
	const PointSet3& points, const std::vector<Viewpoint>& traj,
	const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
	const double viewDis, const double maxAngle, const double maxDis, DroneScanViz* v_viz = nullptr,
	const bool& modl = true
);