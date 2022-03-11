#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include "common_util.h"
#include "building.h"
#include "map_util.h"
#include <glog/logging.h>

struct Trajectory_params
{
	double view_distance;
	double z_up_bounds;
	double z_down_bounds;
	bool double_flag;
	bool split_flag;
	double step;
	bool with_continuous_height;
	bool with_erosion;
	double fov; //Small one in degree
	double vertical_overlap;
	double horizontal_overlap;
	double split_overlap;
};

Eigen::Vector2d
lonLat2Mercator(
	const Eigen::Vector2d& lonLat
);

Eigen::Vector2d
mercator2lonLat(
	const Eigen::Vector2d& mercator
);

Eigen::Vector3d
wgs2cgcs2000(
	const Eigen::Vector3d& v_wgs
);

void
write_unreal_path(
	const std::vector<MyViewpoint>& v_trajectories, const std::string& v_path
);

void
write_smith_path(
	const std::vector<MyViewpoint>& v_trajectories,	const std::string& v_path
);

void
write_normal_path_with_flag(
	const std::vector<MyViewpoint>& v_trajectories,
	const std::string& v_path
);

void
write_normal_path(
	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& v_trajectories, const std::string& v_path
);


// Second element of the pair is the focus point
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
interpolate_path(
	const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& v_trajectories
);

// Second element of the pair is the direction
std::vector<MyViewpoint>
simplify_path_reduce_waypoints(
	const std::vector<MyViewpoint>& v_trajectories
);

void
write_wgs_path(
	const Json::Value& v_args, const std::vector<MyViewpoint>& v_trajectories, const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
read_unreal_trajectory(
	const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
read_normal_trajectory(
	const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
read_hui_trajectory(
	const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
read_smith_trajectory(
	const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>>
read_wgs84_trajectory(
	const std::string& v_path
);

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>
read_smith_spline_trajectory(
	const std::string& v_path
);

double
evaluate_length(
	const std::vector<MyViewpoint> v_trajectory
);

bool
ensure_no_views_less_than_three_meter(
	const Eigen::Vector3d& v_pos, const std::vector<Eigen::Vector3d>& v_exists_pos
);

std::vector<MyViewpoint>
ensure_three_meter_dji(
	const std::vector<MyViewpoint>& v_trajectory,
	const modeltools::Height_map& v_height_map, const float v_safe_distance
);

std::vector<MyViewpoint>
ensure_global_safe(
	const std::vector<MyViewpoint>& v_trajectory, const modeltools::Height_map& v_height_map,
	const float v_safe_distance, const Polygon2& v_boundary

);
std::vector<MyViewpoint>
ensure_safe_trajectory(
	const std::vector<MyViewpoint>& v_trajectory,
	const modeltools::Height_map& v_height_map, const double v_safe_distance
);


std::vector<MyViewpoint>
find_short_cut(
	const std::vector<MyViewpoint>& v_trajectory, const modeltools::Height_map& v_height_map,
	const float v_safe_distance, const Building& v_cur_building, const Tree& v_tree
);

void cluster_duplicate(
	std::vector<MyViewpoint>& v_trajectory,
	const float v_vertical_overlap, const float v_fov, const float v_view_distance
);

std::vector<MyViewpoint>
generate_trajectory(
	const Json::Value& v_params, std::vector<Building>& v_buildings,
	const modeltools::Height_map& v_height_map, const float v_vertical_step,
	const float horizontal_step, const float split_min_distance, const Tree& v_tree
);

bool
ifNeighbouring(
	const Building& a, const Building& b, const double& threshold
);

std::vector<MyViewpoint> generate_trajectory_tg(
	const Json::Value& v_params, std::vector<Building>& v_buildings,
	const modeltools::Height_map& v_height_map, const double v_vertical_step,
	const double horizontal_step, const double split_min_distance, const Tree& v_tree
);

void
generate_distance_map(
	const cv::Mat& v_map, cv::Mat& distance_map,
	const Eigen::Vector2i goal, Eigen::Vector2i now_point, int distance
);

void
update_obstacle_map(
	const cv::Mat& v_map, cv::Mat& distance_map,
	const Eigen::Vector2i goal, Eigen::Vector2i now_point, int distance
);

void
print_map(const cv::Mat& v_map);

void
explore(
	const cv::Mat& v_map, const cv::Mat& distance_map,
	const cv::Mat& obstacle_map, std::vector<Eigen::Vector2i>& trajectory,
	Eigen::Vector2i now_point, const Eigen::Vector2i& goal,
	cv::Mat& visited_map, bool& isFinished, float weight);

std::vector<Eigen::Vector2i>
perform_ccpp(
	const cv::Mat& ccpp_map, const Eigen::Vector2i& v_start_point,
	const Eigen::Vector2i& v_goal, float weight = 1
);
