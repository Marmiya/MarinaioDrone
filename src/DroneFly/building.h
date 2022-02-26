#pragma once
#include "model_tools.h"
#include <CGAL/point_generators_2.h>
#include <CGAL/algorithm.h>
#include <CGAL/random_selection.h>

struct MyViewpoint
{
	Eigen::Vector3d pos_mesh;
	Eigen::Vector3d direction; // Normalized
	Eigen::Vector3d focus_point;
	float pitch; // up -> +pitch
	float yaw;   //forwards->x, towards right -> +yaw
	bool is_towards_reconstruction;

	MyViewpoint(){};
	
	MyViewpoint(const Eigen::Vector3d v_pos_mesh, const Eigen::Vector3d v_focus_point):pos_mesh(v_pos_mesh), focus_point(v_focus_point)
	{
		calculate_direction();
	}

	void calculate_direction()
	{
		direction = (focus_point - pos_mesh).normalized();
		pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) *
			180. / M_PI;
		yaw = std::atan2f(direction[1], direction[0]) * 180. / M_PI;
	}

	
};

struct Building {
	//Eigen::AlignedBox3d bounding_box_3d;
	cgaltools::RotatedBox bounding_box_3d;
	PointSet3 points_camera_space;
	PointSet3 points_world_space;
	CGAL::Bbox_2 bounding_box_2d;
	cv::Vec3b segmentation_color;
	std::vector<cgaltools::RotatedBox> boxes;
	bool is_changed = true;
	bool is_divide = false;
	int parent = -1;
	int one_pass_trajectory_num = 0;
	int closest_trajectory_id = 0;

	//Used for trajectory generation
	int start_box = -1;
	std::vector<MyViewpoint> trajectory;
	std::vector<MyViewpoint> passed_trajectory;

	int find_nearest_trajectory(const Eigen::Vector3d& v_pos) const 
	{
		return std::min_element(trajectory.begin(), trajectory.end(),
			[&v_pos](const MyViewpoint& item1, const MyViewpoint& item2) {
			return (item1.pos_mesh - v_pos).norm() < (item2.pos_mesh - v_pos).norm();
		}) - trajectory.begin();
	}

	int find_nearest_trajectory_2d(const Eigen::Vector3d& v_pos) const {
		return std::min_element(trajectory.begin(), trajectory.end(),
			[&v_pos](const MyViewpoint& item1, const MyViewpoint& item2) {
			Eigen::Vector2d drone_pos(v_pos.x(), v_pos.y());
			Eigen::Vector2d trajectory_pos1(item1.pos_mesh.x(), item1.pos_mesh.y());
			Eigen::Vector2d trajectory_pos2(item1.pos_mesh.x(), item1.pos_mesh.y());
				return (trajectory_pos1 - drone_pos).norm() < (trajectory_pos2 - drone_pos).norm();
		}) - trajectory.begin();
	}
};

struct Next_target {
	int origin_index_in_building_vector = -1;
	int origin_index_in_untraveled_pointset = -1;
	Next_target(const int v_origin_index_in_building_vector, const int v_origin_index_in_untraveled_pointset)
		:origin_index_in_building_vector(v_origin_index_in_building_vector), origin_index_in_untraveled_pointset(v_origin_index_in_untraveled_pointset) {
	}
};

