#pragma once
//
//#include "dronescan.h"
//#include "initialize.h"
//#include "SmithOptimized.h"
//
//
//std::vector<Viewpoint> pathplanning(
//	PointSet3 &points, const SurfaceMesh& mesh, const Eigen::Matrix3d &intrinsicMatrix,
//	const int maxIterTimes, const int initViewNum, const std::string& logPath,
//	const double viewDis, const double maxAngle, const double maxDis
//);
//
//
//void pathplanningAdj(
//	const PointSet3& points, std::vector<Viewpoint>& traj, const std::string& logPath,
//	const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
//	const double viewDis, const double maxAngle, const double maxDis
//);
//
//std::vector<Viewpoint> viewAddtion(
//	const PointSet3& pts, const std::vector<Viewpoint>& traj, const std::string& logPath,
//	const SurfaceMesh& mesh, const modeltools::Height_map& heightMap,
//	const Eigen::Matrix3d& intrinsicMatrix,	const double viewDis, const double maxAngle, const double maxDis
//);