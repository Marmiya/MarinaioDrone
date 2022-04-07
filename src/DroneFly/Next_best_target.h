#pragma once

#include "map_util.h"
#include "DFtrajectory.h"

enum Motion_status { initialization, exploration, reconstruction, done, final_check, reconstruction_in_exploration };

class Next_best_target {
public:
	Motion_status m_motion_status;
	int m_current_building_id = -1;

	double DISTANCE_THRESHOLD;
	std::vector<CGAL::Point_2<K>> sample_points;
	std::vector<cv::Vec3b> region_status;
	std::vector<cv::Vec3b> region_viz_color;

	Eigen::Vector3d m_map_start;
	Eigen::Vector3d m_map_end;

	Polygon2 m_boundary;

	Next_best_target(const Eigen::Vector3d& v_map_start_mesh, const Eigen::Vector3d& v_map_end_mesh, float v_ccpp_cell_distance) :m_map_start(v_map_start_mesh), m_map_end(v_map_end_mesh)
	{
		DISTANCE_THRESHOLD = v_ccpp_cell_distance;
		m_motion_status = Motion_status::initialization;
		for (float y = v_map_start_mesh.y(); y < v_map_end_mesh.y(); y += DISTANCE_THRESHOLD)
			for (float x = v_map_start_mesh.x(); x < v_map_end_mesh.x(); x += DISTANCE_THRESHOLD)
				sample_points.push_back(CGAL::Point_2<K>(x, y));
		region_viz_color = comutil::get_color_table_bgr();
		region_status.resize(sample_points.size(), region_viz_color[0]);
	}

	virtual void get_next_target(int frame_id, const Pos_Pack& v_cur_pos, const std::vector<Building>& v_buildings, bool with_exploration) = 0;

	virtual void update_uncertainty(const Pos_Pack& v_cur_pos, const std::vector<Building>& v_buildings) = 0;

	virtual MyViewpoint determine_next_target(int v_frame_id, const Pos_Pack& v_cur_pos, std::vector<Building>& v_buildings, bool with_exploration, float v_threshold) = 0;
};

class Next_best_target_topology_exploration : public Next_best_target {
public:
	std::vector<Eigen::AlignedBox2d> topology;

	cv::Vec3b color_occupied;
	cv::Vec3b color_unobserved;
	cv::Vec3b color_reconstruction;
	Json::Value m_arg;

	int CCPP_CELL_THRESHOLD;
	int rotation_status = 0;

	int m_current_ccpp_trajectory_id;
	int m_current_color_id;
	int m_isFirst = false;
	std::vector<MyViewpoint> m_ccpp_trajectory;
	std::queue<MyViewpoint> m_exploration_point;

	double memory_y = -1.;
	int dummy1 = 0;
	int dummy2 = 0;
	double dummy3 = 0;

	Next_best_target_topology_exploration(
		const Eigen::Vector3d& v_map_start_mesh, const Eigen::Vector3d& v_map_end_mesh,
		int v_CCPP_CELL_THRESHOLD, const Polygon2& v_boundary, 
		float v_ccpp_cell_distance, const Json::Value& v_arg
	);

	bool get_ccpp_trajectory(
		const Eigen::Vector3d& v_cur_pos, const Building& v_building, int v_ccpp_threshold
	);

	Building default_expand();

	void get_next_target(
		int frame_id, const Pos_Pack& v_cur_pos,
		const std::vector<Building>& v_buildings, bool with_exploration
	) override;

	void update_uncertainty(
		const Pos_Pack& v_cur_pos, const std::vector<Building>& v_buildings
	) override;

	MyViewpoint determine_next_target(
		int v_frame_id, const Pos_Pack& v_cur_pos, std::vector<Building>& v_buildings,
		bool with_exploration, float v_threshold
	) override;
};


/*
 * Ablation study
 */
class Next_best_target_random_min_distance :public Next_best_target
{
public:
	int m_current_exploration_id = -1;
	cv::Vec3b color_unobserved, color_free, color_occupied;
	std::vector<bool> already_explored;

	Next_best_target_random_min_distance(
		const Eigen::Vector3d& v_map_start_mesh, const Eigen::Vector3d& v_map_end_mesh,
		float v_ccpp_cell_distance
	);

	void get_next_target(
		int frame_id, const Pos_Pack& v_cur_pos,
		const std::vector<Building>& v_buildings, bool with_exploration
	) override;
	

	void update_uncertainty(
		const Pos_Pack& v_cur_pos, const std::vector<Building>& v_buildings
	) override;


	MyViewpoint determine_next_target(
		int v_frame_id, const Pos_Pack& v_cur_pos,
		std::vector<Building>& v_buildings, bool with_exploration, float v_threshold
	) override;

};

class Next_best_target_exploration_first :public Next_best_target_topology_exploration
{
public:
	Next_best_target_exploration_first(
		const Eigen::Vector3d& v_map_start_mesh, const Eigen::Vector3d& v_map_end_mesh,
		int v_CCPP_CELL_THRESHOLD, const Polygon2& m_boundary, float v_ccpp_cell_distance,
		const Json::Value& v_arg
	);

	void get_next_target(
		int frame_id, const Pos_Pack& v_cur_pos,
		const std::vector<Building>& v_buildings, bool with_exploration
	) override;
	
	MyViewpoint determine_next_target(
		int v_frame_id, const Pos_Pack& v_cur_pos, std::vector<Building>& v_buildings,
		bool with_exploration, float v_threshold
	) override;
	
};