#pragma once
#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <tinyxml2.h>

#include "viewpoint.h"

/*
 * Tool Function
 */

Eigen::Vector2d lonLat2Mercator(const Eigen::Vector2d& lonLat);

Eigen::Vector2d mercator2lonLat(const Eigen::Vector2d& mercator);

/*
 * IO Function
 */

std::vector<Viewpoint> read_wgs84_trajectory(const std::string& v_path,const Eigen::Vector3d& v_original_wgs);

std::vector<Viewpoint> read_smith18_path(const std::string& v_path);

std::vector<Viewpoint> read_normal_path(const std::string& v_path);

std::vector<Viewpoint> read_5_camera_pose(const fs::path& v_path);

// Thanks for Ke's code
std::vector<Viewpoint> read_maya_camera_pose(const fs::path& v_path);

// Thanks for Yue's code
std::vector<Viewpoint> read_xml_from_cc_camera_pose(
	const Eigen::Vector3d v_origin, // Need the origin
	std::vector<std::string>& pose_xml_filenames,
	bool v_read_sfm_result = true
);

void write_wgs_path(const fs::path& v_root,const std::vector<Viewpoint>& v_trajectories,const Eigen::Vector3d& v_original_wgs);

// TODO
void write_wgs_path_route(const Eigen::Vector3d& v_origin_wgs, const RouteViewpoint& v_trajectories, const std::string& v_path);

void write_smith18_path(const std::vector<Viewpoint>& v_trajectories, const std::string& v_path);

// Used for RC align
void write_normal_path(const std::vector<Viewpoint>& v_trajectories, const std::string& v_path);

#endif
