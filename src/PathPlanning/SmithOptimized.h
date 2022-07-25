#pragma once
#include "common_util.h"
#include "model_tools.h"
#include "metrics.h"

#include <cstdlib>
#include <unordered_set>
#include <corecrt_math_defines.h>
#include <iomanip>

#include "viewpoint.h"
#include "SmithOptCuda.cuh"
#include "SmithViz.h"

extern enum ompStatus;
extern double harmony;
extern double hallmark;
extern double upperHallmark;
extern double steplength;
extern int outerIternum;
extern int innerItrnum;

std::pair<double, Eigen::Vector3d> maxREC(
	const Eigen::Vector3d& postion,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const RTCScene& v_embree_scene,
	const std::vector<std::unordered_set<int>>& watchingVs,
	const double v_dmax,const double v_fov_degree,
	const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans
);

std::pair<double, Eigen::Vector3d> maxUTL(
	const Eigen::Vector3d& postion,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const RTCScene& v_embree_scene,
	const std::vector<std::unordered_set<int>>& watchingVs, const std::vector<double> recpts,
	const double v_dmax, const double v_fov_degree,
	const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans
);

std::tuple<Eigen::Vector3d, Eigen::Vector3d, ompStatus> nelderMeadMethod(
	const Eigen::Vector3d& ori, const double& orirec,
	const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans,
	const std::vector<std::unordered_set<int>>& watchingVs,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const double v_fov_degree,
	const RTCScene& v_embree_scene
);

std::tuple<Eigen::Vector3d, Eigen::Vector3d, ompStatus> nelderMeadMethodPREC(
	const Eigen::Vector3d& ori, const double& orirec, const std::vector<double>& recpts,
	const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans,
	const std::vector<std::unordered_set<int>>& watchingVs,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const double v_fov_degree,
	const RTCScene& v_embree_scene, const modeltools::Height_map& heightMap
);

/*
 * Old interface, for stability of old code.
 * If you are configuring new system, please use new interface for efficiency.
 */

std::vector<Viewpoint> adjustTraj(
	const std::vector<Viewpoint>& traj, const modeltools::Height_map& heightMap,
	const PointSet3& pts, const SurfaceMesh& mesh,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const double v_fov_degree,
    SmithViz* v_viz = nullptr
);

/*
 * New interface.
 */

extern int candidateViewsDens;


std::vector<Viewpoint> SmithAdj(
	const std::vector<Viewpoint>& views, const PointSet3& pts,
	const SurfaceMesh& mesh,
	const Eigen::Matrix3d& intrinsicMatrix, const double viewDis, const double fov,
	const int& CVD = 256
);

std::vector<Viewpoint> adjusting(
	const std::vector<Viewpoint>& views, const PointSet3& pts,
	const SurfaceMesh& mesh,
	const Eigen::Matrix3d& intrinsicMatrix, const double viewDis, const double fov
);

void initialization(const SurfaceMesh& mesh, const int& CVD = 256);

std::pair<std::vector<double3>, std::vector<double3>>
viewsInCuda(const std::vector<Viewpoint>& views);

std::pair<std::vector<double3>, std::vector<double3>>
ptsInCuda(const PointSet3& pts);