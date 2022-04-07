#pragma once

#include <regex>
//#include <cpr/cpr.h>

#include "airsim_control.h"

#include "DFtrajectory.h"

using RotatedBox = cgaltools::RotatedBox;

extern const cv::Vec3b BACKGROUND_COLOR;
extern const cv::Vec3b SKY_COLOR;
extern const int MAX_FEATURES;

class Unreal_object_detector
{
public:
	Unreal_object_detector() = default;

	void get_bounding_box(
		std::map<std::string, cv::Mat>& v_img, std::vector<cv::Vec3b>& v_color_map,
		std::vector<Building>& v_buildings
	);
};

class Real_object_detector
{
public:
	Real_object_detector() = default;

	std::vector<cv::Rect2f>
		process_results(
			std::string input, int cols, int rows
		);

	cv::Vec3b stringToVec3b(std::string input);

	std::string Vec3bToString(cv::Vec3b color)
	{
		return std::to_string(color.val[0]) + " " + std::to_string(color.val[1]) + " " + std::to_string(color.val[2]);
	}

	void get_bounding_box(
		std::map<std::string, cv::Mat>& v_img, std::vector<cv::Vec3b>& v_color_map,
		std::vector<Building>& v_buildings
	);
};
/*
 * Mapper
 */
class Mapper
{
public:
	Json::Value m_args;
	Polygon2 m_boundary;

	Mapper(const Json::Value& v_args);

	virtual void get_buildings(
		std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
		int v_cur_frame_id, modeltools::Height_map& v_height_map
	) = 0;

};
/*
 * GTMapper
 */
class GT_mapper : public Mapper
{
public:
	std::vector<Building> m_buildings_target;
	std::vector<Building> m_buildings_safe_place;

	GT_mapper(const Json::Value& args);

	void get_buildings(
		std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
		const int v_cur_frame_id, modeltools::Height_map& v_height_map
	) override;
	
};
/*
 * Graduate GT Mapper
 */
class Graduate_GT_mapper : public Mapper
{
public:
	std::vector<Building> m_buildings_target;
	std::vector<bool> m_is_building_add;
	std::vector<Building> m_buildings_safe_place;

	Graduate_GT_mapper(const Json::Value& args);

	void get_buildings(
		std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
		const int v_cur_frame_id, modeltools::Height_map& v_height_map
	) override;
	
};
/*
 * Virtual Mapper
 */
class Virtual_mapper : public Mapper
{
public:
	Unreal_object_detector* m_unreal_object_detector;
	Airsim_tools* m_airsim_client;
	std::map<cv::Vec3b, std::string> m_color_to_mesh_name_map;
	// Read Mesh
	std::map<string, PointSet3> m_point_clouds;
	std::map<string, SurfaceMesh> m_meshes;

	struct ImageCluster {
		CGAL::Bbox_2 box;
		std::string name;
		cv::Vec3b color;
		std::vector<int> xs;
		std::vector<int> ys;
	};

	Virtual_mapper(
		const Json::Value& args, Airsim_tools* v_airsim_client, std::map<cv::Vec3b, std::string> color_to_mesh_name_map
	) : Mapper(args), m_airsim_client(v_airsim_client)
	{
		m_unreal_object_detector = new Unreal_object_detector;
		m_color_to_mesh_name_map = color_to_mesh_name_map;
		read_mesh(m_args["mesh_root"].asString(), m_point_clouds, m_meshes);
	}

	std::vector<ImageCluster>
		solveCluster(
			const cv::Mat& vSeg, const std::map<cv::Vec3b, std::string> colorMap, bool& isValid
		);
	
	std::pair<cv::RotatedRect, Point2>
		get_bbox_3d(const PointSet3& v_point_cloud);
	

	float calculate_3d_iou(
		const Building& building1, const Building& building2
	);

	void read_mesh(
		std::string in_path, std::map<string, PointSet3>& v_out_point_clouds,
		std::map<string, SurfaceMesh>& v_out_meshes
	);

	void get_buildings(
		std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
		const int v_cur_frame_id, modeltools::Height_map& v_height_map
	) override;

};
/*
 * Real Mapper
 */
class Real_mapper : public Mapper
{
public:
	Real_object_detector* m_real_object_detector;
	cv::Ptr<cv::Feature2D> orb;
	Airsim_tools* m_airsim_client;

	Real_mapper(
		const Json::Value& args, Airsim_tools* v_airsim_client
	) : Mapper(args), m_airsim_client(v_airsim_client)
	{
		m_real_object_detector = new Real_object_detector;
		orb = cv::ORB::create(MAX_FEATURES);
	}

	void get_buildings(
		std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
		const int v_cur_frame_id, modeltools::Height_map& v_height_map
	) override;
};

void calculate_trajectory_intrinsic(
	const Json::Value& v_args, double& horizontal_step,
	double& vertical_step, double& split_min_distance
);