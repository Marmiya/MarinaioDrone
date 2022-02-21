#pragma once
#ifndef METRICS_H
#define METRICS_H

#include "common_util.h"
#include <iostream>
#include <fstream>
#include <list>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Core>
#include <algorithm>
#include <glog/logging.h>

#include "viewpoint.h"
#include "cgal_tools.h"
#include "model_tools.h"
#include "intersection_tools.h"
#include "map_util.h"

template <typename TYPE>
void print_vector_distribution(const std::vector<TYPE>& reconstructability_map)
{
	
	TYPE average_recon = std::accumulate(reconstructability_map.begin(), reconstructability_map.end(), 0.) / reconstructability_map.
		size();
	TYPE max_recon = (*std::max_element(reconstructability_map.begin(), reconstructability_map.end()));
	TYPE min_recon = (*std::min_element(reconstructability_map.begin(), reconstructability_map.end()));
	std::vector<TYPE> reconstructability_map_vector(reconstructability_map.begin(), reconstructability_map.end());
	std::nth_element(reconstructability_map_vector.begin(),
	                 reconstructability_map_vector.begin() + reconstructability_map_vector.size() / 2,
	                 reconstructability_map_vector.end());
	TYPE quarter_recon = (reconstructability_map_vector[reconstructability_map_vector.size() / 4 * 1]);
	TYPE median_recon = (reconstructability_map_vector[reconstructability_map_vector.size() / 4 * 2]);
	TYPE three_quarter_recon = (reconstructability_map_vector[reconstructability_map_vector.size() / 4 * 3]);
	int number_zero=0;
	for(const auto& item: reconstructability_map_vector) if(item == 0.) number_zero++;

	LOG(INFO) << (boost::format("Average value: %d; Lowest: %d; Highest: %d; Quarter: %d; Median: %d;Three quarter: %d; %d/%d point has value 0")
		% average_recon
		% min_recon
		% max_recon
		% quarter_recon
		% median_recon
		% three_quarter_recon
		% number_zero
		% reconstructability_map_vector.size()
	).str();
}
// it's wrong
std::vector<std::vector<bool>> compute_visibility(const std::vector<Viewpoint>& v_viewpoints,const SurfaceMesh& v_mesh, const PointSet3& point_set,
                                                  const double v_fov_degree_h, const double v_fov_degree_v, const double v_dmax = 99999.,Tree* v_tree=nullptr);

std::vector<std::vector<bool>> compute_visibility(const std::vector<Viewpoint>& v_viewpoints,
	const SurfaceMesh& v_mesh, const PointSet3& point_set,const Eigen::Matrix3d& v_intrinsic_matrix, const double v_dmax = 99999.,
	 Tree* v_tree=nullptr);

std::vector<std::vector<bool>> compute_visibility(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const RTCScene& v_embree_scene);

std::vector<std::vector<int>> compute_visibilityIndex(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, Tree* v_tree);

std::vector<std::vector<int>> compute_visibilityIndex(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const RTCScene& v_embree_scene);

/*
 * Smith18
 */

std::array<double, 5> calculate_point_reconstructability(const Viewpoint& v_view1, const Viewpoint& v_view2,
    const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double dmax);

double biviewsREC(const Eigen::Vector3d& v_view1, const Eigen::Vector3d& v_view2,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double dmax);

std::vector<std::array<double, 5>> reconstructability_hueristic(std::vector<Viewpoint> trajectory,
    const PointSet3& point_set,
    const SurfaceMesh& v_mesh, std::vector<std::vector<bool>>& point_view_visibility,const double dmax = 120., const double v_fov_degree_h=62., const double v_fov_degree_v=42., Tree* tree=nullptr);

/*
 * LaLaLa
 */

const double GSD_weight = 0.2;
const double scale_weight = 2.;
const double disortion_weight = 3.0;
const double redundancy_weight = 0.5;

std::vector<double> compute_reconstructability_lalala_item(const Viewpoint& v_view1, const Viewpoint& v_view2,
    const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double v_max_distance, const double v_fov_degree_h, const double v_fov_degree_v);

void find_min_max_path(std::vector<Eigen::MatrixXd>& v_distance_matrix, const int max_depth);

double compute_reconstructability_lalala(const std::vector<Viewpoint>& v_viewpoints, const Point3& v_point, const Vector3& v_normal, 
                                        const double v_max_distance, const double v_fov_in_degree,  const Eigen::Matrix3d& v_intrinsic_matrix,
	const int v_redundancy_density, const bool v_with_minmax, const bool v_log_txt);

std::vector<double> compute_reconstructability_point_set_lalala(const std::vector<Viewpoint>& v_viewpoints, const PointSet3& v_point_set,
                                                               const std::vector<std::vector<bool>>& point_view_visibility,
                                                               const double v_max_distance, const double v_fov_degree_h, const double v_fov_degree_v,  const Eigen::Matrix3d& v_intrinsic_matrix,
	const int v_redundancy_density, const bool v_with_minmax, const bool v_log_txt);

#endif // !METRICS_H
