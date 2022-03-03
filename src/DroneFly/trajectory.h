#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <unordered_set>
#include <iostream>
#include <fstream>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include<corecrt_math_defines.h>


#include "common_util.h"
#include "building.h"
#include "map_util.h"

Eigen::Vector2d lonLat2Mercator(const Eigen::Vector2d& lonLat) {
	Eigen::Vector2d mercator;
	double x = lonLat.x() * 20037508.34 / 180;
	double y = log(tan((90 + lonLat.y()) * M_PI / 360)) / (M_PI / 180);
	y = y * 20037508.34 / 180;
	mercator = Eigen::Vector2d(x, y);
	return mercator;
}

Eigen::Vector2d mercator2lonLat(const Eigen::Vector2d& mercator) {
	Eigen::Vector2d lonLat;
	double x = mercator.x() / 20037508.34 * 180;
	double y = mercator.y() / 20037508.34 * 180;
	y = 180 / M_PI * (2 * atan(exp(y * M_PI / 180)) - M_PI / 2);
	lonLat = Eigen::Vector2d(x, y);
	return lonLat;
}

Eigen::Vector3d wgs2cgcs2000(const Eigen::Vector3d& v_wgs) {
	Eigen::Matrix3d converter;
	converter << 0.999997079, 3.47778126e-7, -2.6082455e-7, 3.21041821e-8, 1, 2.14655547e-8, 2.13904843e-7, -3.436997e-8, 1;

	Eigen::Vector3d cgcs2000 = converter * v_wgs;
	return cgcs2000;
}


void write_unreal_path(const std::vector<MyViewpoint>& v_trajectories,
                       const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i)
	{
		const Eigen::Vector3d& position = v_trajectories[i].pos_mesh * 100;
		const Eigen::Vector3d& direction = v_trajectories[i].direction;
		boost::format fmt("%04d.png,%s,%s,%s,%s,0,%s\n");
		float pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) *
			180. / M_PI;
		float yaw = std::atan2f(direction[1], direction[0]) * 180. / M_PI;

		pose << (fmt % i % position[0] % -position[1] % position[2] % -pitch % -yaw).str();
	}

	pose.close();
}

void write_smith_path(const std::vector<MyViewpoint>& v_trajectories,
                      const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i)
	{
		const Eigen::Vector3d& position = v_trajectories[i].pos_mesh * 100;
		const Eigen::Vector3d& direction = v_trajectories[i].direction;
		boost::format fmt("%04d.png,%s,%s,%s,%s,0,%s\n");
		float pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) *
			180. / M_PI;
		float yaw = std::atan2f(direction[1], direction[0]) * 180. / M_PI;
		yaw = 90-yaw;
		pose << (fmt % i % -position[0] % position[1] % position[2] % -pitch % yaw).str();
	}

	pose.close();
}

void write_normal_path_with_flag(const std::vector<MyViewpoint>& v_trajectories,
	const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i) {
		const Eigen::Vector3d& position = v_trajectories[i].pos_mesh;
		const Eigen::Vector3d& direction = v_trajectories[i].direction;
		boost::format fmt("%04d.png,%s,%s,%s,%s,0,%s,%s\n");
		pose << (fmt % i % position[0] % position[1] % position[2] % v_trajectories[i].pitch % v_trajectories[i].yaw % v_trajectories[i].is_towards_reconstruction).str();
	}

	pose.close();
}

void write_normal_path(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& v_trajectories,
                       const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i)
	{
		const Eigen::Vector3d& position = v_trajectories[i].first;
		const Eigen::Vector3d& direction = v_trajectories[i].second;
		boost::format fmt("%04d.png,%s,%s,%s,%s,0,%s\n");
		float pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) *
			180. / M_PI;
		float yaw = std::atan2f(direction[1], direction[0]) * 180. / M_PI;

		pose << (fmt % i % position[0] % position[1] % position[2] % pitch % yaw).str();
	}

	pose.close();
}

// Second element of the pair is the focus point
std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> interpolate_path(const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& v_trajectories) {
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> interpolated_trajectory;
	for (int i = 0; i < v_trajectories.size()-1; ++i) {
		const Eigen::Vector3d& position = v_trajectories[i].first;
		const Eigen::Vector3d& next_position = v_trajectories[i+1].first;
		const Eigen::Vector3d& next_focus = v_trajectories[i+1].second;
		Eigen::Vector3d towards = next_position-position;
		int num = towards.norm() / 3.f;
		towards.normalize();
		for(int i_interpolate=0;i_interpolate<num;++i_interpolate)
		{
			interpolated_trajectory.push_back(std::make_pair(
				position + towards * i_interpolate,
				next_focus
			));
		}
	}
	interpolated_trajectory.push_back(v_trajectories.back());
	return interpolated_trajectory;
}

// Second element of the pair is the direction
std::vector<MyViewpoint> simplify_path_reduce_waypoints(const std::vector<MyViewpoint>& v_trajectories) {
	std::vector<MyViewpoint> simplified_trajectory;
	int i0 = 0;
	int i1 = 1;
	Eigen::Vector3d towards(0.f, 0.f, 0.f);
	simplified_trajectory.push_back(v_trajectories[0]);
	while(i1 < v_trajectories.size())
	{
		const Eigen::Vector3d& position0 = v_trajectories[i1-1].pos_mesh;
		const Eigen::Vector3d& position1 = v_trajectories[i1].pos_mesh;
		Eigen::Vector3d next_towards = (position1 - position0).normalized();
		if(towards== Eigen::Vector3d (0.f, 0.f, 0.f))
		{
			i1 += 1;
			towards = next_towards;
		}
		else if (towards.dot(next_towards)>0.99f && v_trajectories[i0].direction.dot(v_trajectories[i1].direction)>0.95)
			i1 += 1;
		else
		{
			if(i1-1!=i0)
				simplified_trajectory.push_back(v_trajectories[i1-1]);
			simplified_trajectory.push_back(v_trajectories[i1]);
			towards = next_towards;
			i0 = i1;
			i1 += 1;
		}
	}
	//simplified_trajectory.push_back(v_trajectories.back());
	return simplified_trajectory;
}

void write_wgs_path(const Json::Value& v_args,const std::vector<MyViewpoint>& v_trajectories,const std::string& v_path) {
	//Eigen::Vector2d origin_wgs(113.92332,22.64429); // Yingrenshi
	Eigen::Vector3d origin_wgs(v_args["geo_origin"][0].asFloat(), v_args["geo_origin"][1].asFloat(), 0.f);
	//Eigen::Vector2d origin_xy=lonLat2Mercator(origin_wgs);
	Eigen::Vector2d origin_xy(origin_wgs.x(), origin_wgs.y());
	std::ofstream pose_total(v_path+"camera_wgs.txt");
	std::ofstream pose(v_path+"camera_wgs_0.txt");

	Eigen::Vector3d prev_pos = v_trajectories[0].pos_mesh;
	int cur_log_id = 0;
	for (int i_id = 0; i_id < v_trajectories.size(); i_id++) {
		const Eigen::Vector3d& position = v_trajectories[i_id].pos_mesh;
		Eigen::Vector2d pos_mac = Eigen::Vector2d(position.x(), position.y()) + origin_xy;
		Eigen::Vector2d pos_wgs = mercator2lonLat(pos_mac);
		const Eigen::Vector3d& direction = v_trajectories[i_id].direction;

		boost::format fmt("%f %f %f %f %f\n");
		float pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) / M_PI * 180.f;
		float yaw = std::atan2f(direction[1], direction[0]) / M_PI * 180.f;
		yaw = - yaw + 90.f;
		if (yaw > 180.f)
			yaw -= 360.f;

		float z = position[2] + origin_wgs.z();
		//if (z > 119)
		//	z = 119;
		pose << (fmt % pos_wgs[0] % pos_wgs[1] % z % yaw % pitch).str();
		pose_total << (fmt % pos_wgs[0] % pos_wgs[1] % z % yaw % pitch).str();
		if((position-prev_pos).norm()>500)
		{
			pose.close();
			pose = std::ofstream(v_path + "camera_wgs_" + std::to_string(cur_log_id + 1) + ".txt");
			cur_log_id += 1;
		}
		prev_pos = position;
	}
	pose.close();
	pose_total.close();
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_unreal_trajectory(const std::string& v_path)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		float pitch = -std::atof(tokens[4].c_str());
		float yaw = -std::atof(tokens[6].c_str());

		float dx = 1.f;
		float dy = std::tanf(yaw / 180.f * M_PI) * dx;
		float dz = std::sqrtf(dx * dx + dy * dy) * std::tanf(pitch / 180.f * M_PI);

		Eigen::Vector3d direction(dx, dy, dz);

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(std::atof(tokens[1].c_str()), -std::atof(tokens[2].c_str()),
			                std::atof(tokens[3].c_str())) / 100,
			direction.normalized()
		));
	}
	while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_normal_trajectory(const std::string& v_path)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		float pitch = std::atof(tokens[4].c_str());
		float yaw = std::atof(tokens[6].c_str());

		float dz = std::sin(pitch / 180.f * M_PI);
		float dxdy = std::cos(pitch / 180.f * M_PI);
		float dy = std::sin(yaw / 180.f * M_PI) * dxdy;
		float dx = std::cos(yaw / 180.f * M_PI) * dxdy;

		Eigen::Vector3d direction(dx, dy, dz);

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(std::atof(tokens[1].c_str()), std::atof(tokens[2].c_str()), std::atof(tokens[3].c_str())),
			direction.normalized()
		));
	}
	while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_hui_trajectory(const std::string& v_path) {
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do {
		std::getline(pose, line);
		if (line.size() < 3) {
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(std::atof(tokens[0].c_str()), std::atof(tokens[1].c_str()), std::atof(tokens[2].c_str())),
			Eigen::Vector3d(0, 0, -1)
		));
	} while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_smith_trajectory(const std::string& v_path) {
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do {
		std::getline(pose, line);
		if (line.size() < 3) {
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		float pitch = -std::atof(tokens[4].c_str());
		float yaw = 90-std::atof(tokens[6].c_str());

		float dz = std::sin(pitch / 180.f * M_PI);
		float dxdy = std::cos(pitch / 180.f * M_PI);
		float dy = std::sin(yaw / 180.f * M_PI) * dxdy;
		float dx = std::cos(yaw / 180.f * M_PI) * dxdy;

		Eigen::Vector3d direction(dx, dy, dz);

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(-std::atof(tokens[1].c_str())/100, std::atof(tokens[2].c_str()) / 100, std::atof(tokens[3].c_str()) / 100) ,
			direction.normalized()
		));
	} while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>> read_wgs84_trajectory(const std::string& v_path)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector2d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(" "));

		float pitch = std::atof(tokens[4].c_str());
		float yaw = std::atof(tokens[3].c_str());

		float longitude = std::atof(tokens[0].c_str());
		float latitude = std::atof(tokens[1].c_str());
		float z = std::atof(tokens[2].c_str());

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(longitude, latitude, z),
			Eigen::Vector2d(pitch,yaw)
		));
	}
	while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> read_smith_spline_trajectory(const std::string& v_path)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";
	std::string t;
	std::getline(pose, t);

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		float z = std::atof(tokens[2].c_str());
		float x = std::atof(tokens[0].c_str());
		float y = std::atof(tokens[1].c_str());

		o_trajectories.push_back(std::make_pair(
			Eigen::Vector3d(x, y, z),
			Eigen::Vector3d(0.f, 0.f, 0.f)
		));
	}
	while (!pose.eof());

	pose.close();
	return o_trajectories;
}

float evaluate_length(const std::vector<MyViewpoint> v_trajectory) {
	float total_length = 0;
	if (v_trajectory.size() < 2)
		return 0;
	for (int idx = 0; idx < v_trajectory.size() - 1; ++idx) {
		total_length += (v_trajectory[idx + 1].pos_mesh - v_trajectory[idx].pos_mesh).norm();
	}

	return total_length;
}

bool ensure_no_views_less_than_three_meter(const Eigen::Vector3d& v_pos,const std::vector<Eigen::Vector3d>& v_exists_pos)
{
	for(const auto& item:v_exists_pos)
	{
		if((item-v_pos).norm() < 3.f)
			return false;
	}
	return true;
}

std::vector<MyViewpoint> ensure_three_meter_dji(const std::vector<MyViewpoint>& v_trajectory, const modeltools::Height_map& v_height_map, const float v_safe_distance) {
	std::vector<MyViewpoint> safe_trajectory;
	std::vector<Eigen::Vector3d> exists_pos;
	for (int i = 0; i < v_trajectory.size(); ++i) {
		const auto& cur_item = v_trajectory[i];
		
		if (ensure_no_views_less_than_three_meter(cur_item.pos_mesh,exists_pos))
		{
			safe_trajectory.push_back(cur_item);
			exists_pos.push_back(cur_item.pos_mesh);
		}
		else
		{
			std::uniform_real_distribution<float> ud(0.f,1.f);
			std::random_device rd;
			std::mt19937 gen(rd());
			const float search_radius = 10.f;
			bool accept=false;
			MyViewpoint v;
			while(!accept)
			{
				v.pos_mesh=Eigen::Vector3d(
					cur_item.pos_mesh.x()+(ud(gen)-.5f)*2*search_radius,
					cur_item.pos_mesh.y()+(ud(gen)-.5f)*2*search_radius,
					cur_item.pos_mesh.z()+(ud(gen))*search_radius);
				v.focus_point=cur_item.focus_point;
				v.direction=(v.focus_point-v.pos_mesh).normalized();
				if (v_height_map.in_bound(v.pos_mesh.x(), v.pos_mesh.y()) && v_height_map.get_height(v.pos_mesh.x(), v.pos_mesh.y()) + v_safe_distance < v.pos_mesh.z())
					accept = ensure_no_views_less_than_three_meter(v.pos_mesh,exists_pos);
			}
			safe_trajectory.push_back(v);
			exists_pos.push_back(v.pos_mesh);
		}
	}
	return safe_trajectory;
}

std::vector<MyViewpoint> ensure_global_safe(
	const std::vector<MyViewpoint>& v_trajectory,
	const modeltools::Height_map& v_height_map, const float v_safe_distance,const Polygon2& v_boundary)
{
	std::vector<MyViewpoint> safe_trajectory;
	std::vector<MyViewpoint> boundary_safe_trajectory = v_trajectory;

	//// Ensure boundary safe
	//for (int i = 0; i < v_trajectory.size() - 1; ++i) {
	//	boundary_safe_trajectory.push_back(v_trajectory[i]);
	//	Eigen::Vector2d cur_item = Eigen::Vector2d(v_trajectory[i].first.x(), v_trajectory[i].first.y());
	//	float cur_z = v_trajectory[i].first.z();
	//	auto next_item = Eigen::Vector2d(v_trajectory[i + 1].first.x(), v_trajectory[i + 1].first.y());
	//	float length_2D = (cur_item - next_item).norm();
	//	Eigen::Vector2d temp_point = cur_item;
	//	bool accept;
	//	bool fail = false;

	//	while ((cur_item - next_item).norm() > 5)
	//	{
	//		cur_item = temp_point;
	//		Eigen::Vector2d direction = (next_item - cur_item).normalized();
	//		for (int angle_in_degree = 0; angle_in_degree <= 90; angle_in_degree+=10)
	//		{
	//			// Add angle
	//			float now_angle = std::atan2(direction.y(), direction.x()) + (angle_in_degree / 180 * M_PI);
	//			Eigen::Vector2d now_direction = Eigen::Vector2d(std::cos(now_angle), std::sin(now_angle));
	//			while ((cur_item - temp_point).norm() < length_2D && (cur_item - next_item).norm() > 5)
	//			{
	//				cur_item += now_direction * 2;
	//				accept = true;
	//				if (v_boundary.bounded_side(CGAL::Point_2<K>(cur_item.x(), cur_item.y())) != CGAL::ON_BOUNDED_SIDE)
	//				{
	//					accept = false;
	//					cur_item = temp_point;
	//					break;
	//				}
	//			}
	//			if (accept)
	//			{
	//				boundary_safe_trajectory.push_back(std::make_pair(Eigen::Vector3d(cur_item.x(), cur_item.y(), cur_z), v_trajectory[i].second));
	//				temp_point = cur_item;
	//				break;
	//			}

	//			// Sub angle
	//			now_angle = std::atan2(direction.y(), direction.x()) - (angle_in_degree / 180 * M_PI);
	//			now_direction = Eigen::Vector2d(std::cos(now_angle), std::sin(now_angle));
	//			while ((cur_item - temp_point).norm() < length_2D && (cur_item - next_item).norm() > 5)
	//			{
	//				cur_item += now_direction * 2;
	//				accept = true;
	//				if (v_boundary.bounded_side(CGAL::Point_2<K>(cur_item.x(), cur_item.y())) != CGAL::ON_BOUNDED_SIDE)
	//				{
	//					accept = false;
	//					cur_item = temp_point;
	//					break;
	//				}
	//			}
	//			if (accept)
	//			{
	//				boundary_safe_trajectory.push_back(std::make_pair(Eigen::Vector3d(cur_item.x(), cur_item.y(), cur_z), v_trajectory[i].second));
	//				temp_point = cur_item;
	//				break;
	//			}
	//			else
	//				fail = true;
	//		}
	//		if (fail)
	//			break;
	//	}
	//}

	// Ensure Height Safe
	for (int i = 0; i < boundary_safe_trajectory.size()-1;++i) {
		Eigen::Vector3d cur_item = boundary_safe_trajectory[i].pos_mesh;
		auto next_item = boundary_safe_trajectory[i + 1];


		if (v_height_map.in_bound(boundary_safe_trajectory[i].pos_mesh.x(), boundary_safe_trajectory[i].pos_mesh.y()) && v_height_map.get_height(boundary_safe_trajectory[i].pos_mesh.x(), boundary_safe_trajectory[i].pos_mesh.y()) + v_safe_distance > boundary_safe_trajectory[i].pos_mesh.z())
			boundary_safe_trajectory[i].pos_mesh[2] = v_height_map.get_height(boundary_safe_trajectory[i].pos_mesh.x(), boundary_safe_trajectory[i].pos_mesh.y()) + v_safe_distance;
		boundary_safe_trajectory[i].calculate_direction();
		if (v_height_map.in_bound(boundary_safe_trajectory[i + 1].pos_mesh.x(), boundary_safe_trajectory[i + 1].pos_mesh.y()) && v_height_map.get_height(boundary_safe_trajectory[i + 1].pos_mesh.x(), boundary_safe_trajectory[i + 1].pos_mesh.y()) + v_safe_distance > boundary_safe_trajectory[i + 1].pos_mesh.z())
			boundary_safe_trajectory[i + 1].pos_mesh[2] = v_height_map.get_height(boundary_safe_trajectory[i + 1].pos_mesh.x(), boundary_safe_trajectory[i + 1].pos_mesh.y()) + v_safe_distance;
		boundary_safe_trajectory[i + 1].calculate_direction();

		safe_trajectory.push_back(boundary_safe_trajectory[i]);
		Eigen::Vector3d direction = (next_item.pos_mesh - cur_item).normalized();
		bool accept = true;
		float top_height = cur_item.z();
		
		while ((cur_item - next_item.pos_mesh).norm() > 5)
		{
			cur_item += direction * 2;

			if (v_height_map.get_height(cur_item.x(), cur_item.y()) + v_safe_distance > cur_item.z())
			{
				accept = false;
				float safe_height = cur_item.z();
				while (v_height_map.get_height(cur_item.x(), cur_item.y()) + v_safe_distance > safe_height) {
					safe_height += 5;
				}
				top_height = top_height > safe_height ? top_height : safe_height;
			}
		}
		if(!accept)
		{
			safe_trajectory.emplace_back(Eigen::Vector3d(boundary_safe_trajectory[i].pos_mesh.x(), boundary_safe_trajectory[i].pos_mesh.y(), top_height), boundary_safe_trajectory[i].focus_point);
			safe_trajectory.back().is_towards_reconstruction = next_item.is_towards_reconstruction;
			safe_trajectory.emplace_back(Eigen::Vector3d(next_item.pos_mesh.x(), next_item.pos_mesh.y(), top_height), next_item.focus_point);
			safe_trajectory.back().is_towards_reconstruction = next_item.is_towards_reconstruction;
		}
		/*
		while((cur_item-next_item.first).norm()>5)
		{
			cur_item += direction * 2;
			if(v_height_map.get_height(cur_item.x(), cur_item.y()) + v_safe_distance > cur_item.z())
			{
				accept = false;
			}
		}
		if(!accept)
		{
			safe_trajectory.emplace_back(Eigen::Vector3d(v_trajectory[i].first.x(), v_trajectory[i].first.y(), next_item.first.z() + 5), v_trajectory[i].second);
			safe_trajectory.emplace_back(Eigen::Vector3d(next_item.first.x(), next_item.first.y(), next_item.first.z() + 5), v_trajectory[i+1].second);
		}*/

	}
	safe_trajectory.push_back(boundary_safe_trajectory.back());
	return safe_trajectory;
}

std::vector<MyViewpoint> ensure_safe_trajectory(
	const std::vector<MyViewpoint>& v_trajectory,
	const modeltools::Height_map& v_height_map,const float v_safe_distance)
{
	std::vector<MyViewpoint> safe_trajectory;
	for (auto item : v_trajectory) {
		if (v_height_map.in_bound(item.pos_mesh.x(), item.pos_mesh.y()) && v_height_map.get_height(item.pos_mesh.x(), item.pos_mesh.y()) + v_safe_distance > item.pos_mesh.z())
			item.pos_mesh[2] = v_height_map.get_height(item.pos_mesh.x(), item.pos_mesh.y()) + v_safe_distance;
		item.calculate_direction();
		safe_trajectory.push_back(item);
	}
	return safe_trajectory;
}

struct Trajectory_params
{
	float view_distance;
	float z_up_bounds;
	float z_down_bounds;
	bool double_flag;
	bool split_flag;
	float step;
	bool with_continuous_height;
	bool with_erosion;
	float fov; //Small one in degree
	float vertical_overlap;
	float horizontal_overlap;
	float split_overlap;
};
/*
bool generate_next_view_curvature(const Trajectory_params& v_params,
                        Building& v_building,
                        std::pair<Eigen::Vector3d, Eigen::Vector3d>& v_cur_pos)
{
	const Eigen::AlignedBox3d* target_box = nullptr;
	float cur_angle = 0.f;
	bool start_flag = false;
	int cur_box_id = -1;
	float angle_step = v_params.xy_angle / 180.f * M_PI;
	// Init
	if (v_cur_pos.first == Eigen::Vector3d(0.f, 0.f, 0.f))
	{
		start_flag = true;
		const Eigen::AlignedBox3d* closest_box;
		while (closest_box != target_box || target_box == nullptr)
		{
			cur_box_id += 1;
			target_box = &v_building.boxes[cur_box_id];
			Eigen::Vector3d next_position(v_cur_pos.first);
			float radius = std::max(target_box->sizes().x(), target_box->sizes().y()) / 2 + v_params.view_distance;
			next_position.x() = radius * std::cos(cur_angle) + target_box->center().x();
			next_position.y() = radius * std::sin(cur_angle) + target_box->center().y();
			closest_box = &*std::min_element(v_building.boxes.begin(), v_building.boxes.end(),
			                                 [&next_position](const Eigen::AlignedBox3d& item1,
			                                                  const Eigen::AlignedBox3d& item2)
			                                 {
				                                 return (next_position - item1.center()).norm() < (next_position - item2
					                                 .center()).norm();
			                                 });;
		}
		v_building.start_box = cur_box_id;
		cur_angle = v_params.xy_angle / 180.f * M_PI;
	}
	else
	{
		target_box = &*std::min_element(v_building.boxes.begin(), v_building.boxes.end(),
		                                [&v_cur_pos](const Eigen::AlignedBox3d& item1, const Eigen::AlignedBox3d& item2)
		                                {
			                                return (v_cur_pos.first - item1.center()).norm() < (v_cur_pos.first - item2.
				                                center()).norm();
		                                });
		cur_box_id = target_box - &v_building.boxes[0];
		Eigen::Vector3d view_to_center = v_cur_pos.first - target_box->center();
		cur_angle = std::atan2f(view_to_center[1], view_to_center[0]);
	}

	float radius = std::max(target_box->sizes().x(), target_box->sizes().y()) / 2 + v_params.view_distance;
	cur_angle -= angle_step;
	// TODO: UGLY
	//if(target_box == &v_building.boxes[0])
	if (!start_flag && v_building.start_box == cur_box_id && std::abs(cur_angle) < angle_step - 1e-6)
		return false;

	//std::cout << cur_box_id << ", " << cur_angle << std::endl;
	Eigen::Vector3d next_position(v_cur_pos.first);
	Eigen::Vector3d camera_focus = target_box->center();

	next_position.x() = radius * std::cos(cur_angle) + target_box->center().x();
	next_position.y() = radius * std::sin(cur_angle) + target_box->center().y();
	v_cur_pos.first = next_position;
	v_cur_pos.second = camera_focus;
	return true;
}
*/

std::vector<MyViewpoint> find_short_cut(
	const std::vector<MyViewpoint>& v_trajectory,
	const modeltools::Height_map& v_height_map, const float v_safe_distance,const Building& v_cur_building, const Tree& v_tree)
{
	//Eigen::Vector3d center = v_cur_building.bounding_box_3d.box.center();
	std::vector<MyViewpoint> safe_trajectory;

	for (auto item : v_trajectory) {
		size_t id_item = &item-&v_trajectory[0];
		Eigen::Vector3d safe_position = item.pos_mesh;

		bool accept = false;
		while(!accept)
		{
			if (v_height_map.in_bound(safe_position.x(), safe_position.y()) && v_height_map.get_height(safe_position.x(), safe_position.y()) + v_safe_distance > safe_position.z())
				safe_position[2] = v_height_map.get_height(safe_position.x(), safe_position.y()) + v_safe_distance;
			
			bool ray_accept=false;
			while(!ray_accept)
			{
				auto source = Point3(safe_position.x(),safe_position.y(),safe_position.z());
				auto destination = Point3(item.focus_point.x(),item.focus_point.y(),item.focus_point.z());
				auto direction = destination-source;
				direction/=std::sqrt(direction.squared_length());
				Ray3 ray(source,direction);
				auto result = v_tree.first_intersection(ray);
				if(!result)
					ray_accept = true;
				else
				{
					const Point3* point=boost::get<Point3>(&(result->first));
					if(point)
					{
						ray_accept = CGAL::squared_distance(source, destination) < CGAL::squared_distance(source, *point)+.01;
						if(!ray_accept)
						{
							Eigen::Vector3d direction(item.direction.x(),item.direction.y(),0);
							direction.normalize();
							safe_position += direction * 1;
						}
					}
					else
						ray_accept = true;
					
				}
			}

			if(v_height_map.get_height(safe_position.x(), safe_position.y()) + v_safe_distance <= safe_position.z())
				accept = true;
		}

		//if(safe_position.z() > 1 * item.pos_mesh.z() && v_height_map.get_height(safe_position.x(), safe_position.y()) > v_cur_building.bounding_box_3d.box.max().z())
		//{
		//	safe_position = item.pos_mesh;
		//	Eigen::Vector3d direction = item.focus_point - safe_position;
		//	direction.z() = 0;
		//	direction.normalize();
		//	direction.z() = 1;

		//	while (v_height_map.get_undilated_height(safe_position.x(), safe_position.y()) + v_safe_distance > safe_position.z()) {
		//		safe_position += direction * 1;

		//		// Abort if the viewpoint is on above the roof
		//		if (v_height_map.get_undilated_height(safe_position.x(), safe_position.y()) == v_cur_building.bounding_box_3d.box.max().z())
		//		{
		//			safe_position -= direction * 1;
		//			break;
		//		}
		//	}
		//}

		item.pos_mesh = safe_position;
		item.calculate_direction();
		safe_trajectory.push_back(item);
	}
	return safe_trajectory;
}

void cluster_duplicate(
	std::vector<MyViewpoint>& v_trajectory,
	const float v_vertical_overlap, const float v_fov,const float v_view_distance)
{
	float duplicate_threshold = 5; // Threshold to cluster views
	std::vector<std::vector<int>> duplicate_pairs; // Store the clusters. Smallest index -> Largest index
	std::vector<int> index_table(v_trajectory.size(),-1); // Whether a view is already been clustered
	
	// Clustering
	for (int i_view1 = 0;i_view1 < v_trajectory.size();i_view1++) {
		for (int i_view2 = 0;i_view2 < i_view1;i_view2++) {
			if((v_trajectory[i_view1].pos_mesh - v_trajectory[i_view2].pos_mesh).norm() < duplicate_threshold)
			{
				if(index_table[i_view2]!=-1) // i_view2 has a cluster
				{
					duplicate_pairs[index_table[i_view2]].push_back(i_view1);
				}
				else // i_view2 has not been cluster yet
				{
					duplicate_pairs.emplace_back(std::vector<int>{i_view2,i_view1});
					index_table[i_view1] = duplicate_pairs.size() - 1;
					index_table[i_view2] = duplicate_pairs.size() - 1;
				}
				break;
			}
		}
	}

	std::vector<bool> delete_flag(v_trajectory.size(), false);
	for(int i_pair = 0; i_pair < duplicate_pairs.size(); ++i_pair)
	{
		const Eigen::Vector3d& pos = v_trajectory[duplicate_pairs[i_pair][0]].pos_mesh;
		const Eigen::Vector3d& first_focus_point= v_trajectory[duplicate_pairs[i_pair][0]].pos_mesh;
		const Eigen::Vector3d& last_focus_point= v_trajectory[duplicate_pairs[i_pair].back()].pos_mesh;
		
		Eigen::Vector3d direction = (first_focus_point - pos).normalized();
		
		float new_angle = -std::atan2(direction.z(), std::sqrtf(direction.x() * direction.x() + direction.y() * direction.y()));
		float surface_distance_one_view = std::tan(new_angle+(v_fov / 2) / 180.f * M_PI) * v_view_distance - std::tan(new_angle-(v_fov / 2) / 180.f * M_PI) * v_view_distance;

		int step = ((last_focus_point - first_focus_point).norm() - surface_distance_one_view) / (surface_distance_one_view * (1 - v_vertical_overlap)) + 1;

		if (step >= duplicate_pairs[i_pair].size()) // Can not be optimized
			continue;
		Eigen::Vector3d current_focus_point = first_focus_point;
		for(int i_view = 1;i_view < duplicate_pairs[i_pair].size();i_view++)
		{
			if(i_view>=step)
			{
				delete_flag[duplicate_pairs[i_pair][i_view]] = true;
			}
			else
			{
				current_focus_point.z() -= surface_distance_one_view;
				v_trajectory[duplicate_pairs[i_pair][i_view]].focus_point = current_focus_point;
				v_trajectory[duplicate_pairs[i_pair][i_view]].calculate_direction();
			}
		}
	}

	v_trajectory.erase(std::remove_if(v_trajectory.begin(), v_trajectory.end(), [&delete_flag, &v_trajectory](auto& item)
		{
			return delete_flag[&item - &v_trajectory[0]];
		}), v_trajectory.end());
}


std::vector<MyViewpoint> generate_trajectory(
	const Json::Value& v_params, std::vector<Building>& v_buildings,
	const modeltools::Height_map& v_height_map, const float v_vertical_step,
	const float horizontal_step, const float split_min_distance, const Tree& v_tree
)
{
	float view_distance = v_params["view_distance"].asFloat();
	float safe_distance = v_params["safe_distance"].asFloat();
	
	// Split building when it cannot be covered by one round flight
	std::vector<bool> building_valid_flags(v_buildings.size(), true);
	
	if (v_params["split_flag"].asBool())
	{
		for (int i_building = v_buildings.size() - 1; i_building >= 0; --i_building)
		{
			auto& item_building = v_buildings[i_building];
			if (!item_building.is_changed)
			{
				continue;
			}
			if (item_building.parent != -1)
			{
				continue;
			}
			int split_width_num = int((item_building.bounding_box_3d.box.max().x() - item_building.bounding_box_3d.box.min().x()) / split_min_distance);
			int split_length_num = int((item_building.bounding_box_3d.box.max().y() - item_building.bounding_box_3d.box.min().y()) / split_min_distance);
			split_width_num += 1;
			split_length_num += 1;
			if (split_width_num < 1)
				split_width_num = 1;
			if (split_length_num < 1)
				split_length_num = 1;
			Building temp_building(item_building);
			if (split_width_num <= 1 && split_length_num <= 1) //Do not need to split
				continue;

			// Delete the exist split building
			for (int i_building2 = v_buildings.size() - 1;i_building2 >= 0; --i_building2)
				if (v_buildings[i_building2].parent == i_building)
					building_valid_flags[i_building2] = false;

			
			item_building.is_divide = true;
			temp_building.parent = i_building;
			Eigen::Vector3d min_vector, max_vector;
			for (int i = 0; i < split_width_num; i++)
			{
				max_vector[2] = item_building.bounding_box_3d.box.max().z();
				min_vector[2] = item_building.bounding_box_3d.box.min().z();
				min_vector[0] = item_building.bounding_box_3d.box.min().x() + i * split_min_distance;
				max_vector[0] = item_building.bounding_box_3d.box.min().x() + (i + 1) * split_min_distance;
				if (i == split_width_num - 1)
					max_vector[0] = item_building.bounding_box_3d.box.max().x();


				
				for (int j = 0; j < split_length_num; j++)
				{
					min_vector[1] = item_building.bounding_box_3d.box.min().y() + j * split_min_distance;
					max_vector[1] = item_building.bounding_box_3d.box.min().y() + (j + 1) * split_min_distance;
					if (j == split_length_num - 1)
						max_vector[1] = item_building.bounding_box_3d.box.max().y();

					Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
					transform.translate(item_building.bounding_box_3d.box.center());
					transform.rotate(Eigen::AngleAxisd(item_building.bounding_box_3d.angle, Eigen::Vector3d::UnitZ()));
					transform.translate(-item_building.bounding_box_3d.box.center());

					cgaltools::RotatedBox box;
					box.angle = item_building.bounding_box_3d.angle;

					Eigen::Vector3d v1 = transform * min_vector;
					Eigen::Vector3d v2 = transform * Eigen::Vector3d(max_vector.x(), min_vector.y(), min_vector.z());
					Eigen::Vector3d v3 = transform * max_vector;

					box.cv_box = cv::RotatedRect(
						cv::Point2f(v1.x(), v1.y()),
						cv::Point2f(v2.x(), v2.y()),
						cv::Point2f(v3.x(), v3.y())
					);

					box.box = Eigen::AlignedBox3d(
						Eigen::Vector3d(box.cv_box.center.x - box.cv_box.size.width / 2, box.cv_box.center.y - box.cv_box.size.height / 2, min_vector.z()),
						Eigen::Vector3d(box.cv_box.center.x + box.cv_box.size.width / 2, box.cv_box.center.y + box.cv_box.size.height / 2, max_vector.z()));

					/*
					LOG(INFO) << box.cv_box.angle / 180. * M_PI;
					LOG(INFO) << box.angle;
					LOG(INFO) << item_building.bounding_box_3d.angle;
					LOG(INFO) << item_building.bounding_box_3d.cv_box.angle / 180. * M_PI;
					*/
					
					temp_building.bounding_box_3d = box;
					v_buildings.push_back(temp_building);
				}
			}
		}
	}

	// BUG Temporal solution. Do not sure
	v_buildings.erase(std::remove_if(v_buildings.begin(), v_buildings.end(), [&building_valid_flags, &v_buildings](const auto& building)
		{
			int idx = &building - &(v_buildings[0]);
			if (idx >= building_valid_flags.size())
				return false;
			else
				return !building_valid_flags[idx];
		}), v_buildings.end());
	
	std::vector<MyViewpoint> total_trajectory;
	for (int id_building = 0; id_building < v_buildings.size(); ++id_building) {
		if(!v_buildings[id_building].is_changed)
			continue;
		if(v_buildings[id_building].is_divide)
		{
			v_buildings[id_building].is_changed = false;
			continue;
		}
		std::vector<MyViewpoint> item_trajectory;

		float xmin = v_buildings[id_building].bounding_box_3d.box.min().x();
		float ymin = v_buildings[id_building].bounding_box_3d.box.min().y();
		float zmin = v_buildings[id_building].bounding_box_3d.box.min().z();
		float xmax = v_buildings[id_building].bounding_box_3d.box.max().x();
		float ymax = v_buildings[id_building].bounding_box_3d.box.max().y();
		float zmax = v_buildings[id_building].bounding_box_3d.box.max().z();

		bool double_flag = v_params["double_flag"].asBool();

		float z_up_bounds = view_distance/std::sqrt(3); //Suppose the top view is toward the top end point of the cube, view angle is 30 degree
		float z_down_bounds = view_distance/std::sqrt(3); //Suppose the bottom view is toward the bottom end point of the cube, view angle is 30 degree
		// Detect if it needs drop
		float fake_vertical_step = v_vertical_step; // When num_pass<=2. the vertical_step will be the height / 2
		int num_pass = 1;
		{
			if (double_flag) {
				num_pass = (zmax + z_up_bounds - z_down_bounds) / v_vertical_step;
				num_pass += 1; // Cell the division
				//if (num_pass <=1)
				//{
				//	num_pass = 2;
				//	fake_vertical_step = (zmax + z_up_bounds) / 2;
				//}
			}
		}

		float focus_z_corner = - view_distance / 3. * std::sqrtf(6.);
		float focus_z_second_corner = - std::sqrtf(15.) / 6. * view_distance;
		float focus_z_corner_normal = - std::sqrtf(3.) / 3 * view_distance;
		
		for (int i_pass = 0; i_pass < num_pass; ++i_pass) {
			float z = std::max(zmax + z_up_bounds - fake_vertical_step * i_pass, z_down_bounds);
			float focus_z = z + focus_z_corner;

			std::vector corner_points{
				Eigen::Vector3d(xmin - view_distance, ymin - view_distance, z),
				Eigen::Vector3d(xmax + view_distance, ymin - view_distance, z),
				Eigen::Vector3d(xmax + view_distance, ymax + view_distance, z),
				Eigen::Vector3d(xmin - view_distance, ymax + view_distance, z),
				Eigen::Vector3d(xmin - view_distance, ymin - view_distance, z)
			};
			std::vector focus_points{
				Eigen::Vector3d(xmin, ymin, focus_z),
				Eigen::Vector3d(xmax, ymin, focus_z),
				Eigen::Vector3d(xmax, ymax, focus_z),
				Eigen::Vector3d(xmin, ymax, focus_z),
				Eigen::Vector3d(xmin, ymin, focus_z)
			};

			Eigen::Vector3d focus_point;
			for(int i_edge = 0;i_edge < corner_points.size()-1;++i_edge)
			{
				Eigen::Vector3d cur_pos = corner_points[i_edge];
				Eigen::Vector3d next_direction = (corner_points[i_edge+1]-corner_points[i_edge]).normalized();
				while(true)
				{
					if(item_trajectory.size()==0 || (focus_points[i_edge]-cur_pos).normalized().dot(next_direction)>0) // Start corner
					{
						focus_point = focus_points[i_edge];
						item_trajectory.emplace_back(cur_pos, focus_point);
						cur_pos += next_direction * horizontal_step;
					}
					else if((cur_pos-corner_points[i_edge+1]).norm()<horizontal_step) // Last point
					{
						cur_pos = corner_points[i_edge+1];
						focus_point = focus_points[i_edge+1];
						item_trajectory.emplace_back(cur_pos, focus_point);
						break;
					}
					else if((focus_points[i_edge+1]-cur_pos).normalized().dot(next_direction)<0) // Last point with direction change
					{
						focus_point = focus_points[i_edge+1];
						item_trajectory.emplace_back(cur_pos, focus_point);
						cur_pos += next_direction * horizontal_step;
					}
					else // Normal
					{
						focus_point = cur_pos+(Eigen::Vector3d(0.f,0.f,1.f).cross(next_direction)) * view_distance;
						focus_point.z() -= view_distance / std::sqrt(3);
						item_trajectory.emplace_back(cur_pos, focus_point);
						cur_pos += next_direction * horizontal_step;
					}
				}
				
			}

			//float delta = (xmax + view_distance - cur_pos.x()) / horizontal_step;
			//delta = delta > 4 ? delta : 4.1;
			//v_buildings[id_building].one_pass_trajectory_num += int(delta);
			//for (int i = 0; i <  delta ;++i)
			//{
			//	if(i==0)
			//	{
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance, view_distance, focus_z_corner);
			//		item_trajectory.emplace_back(cur_pos, focus_point);
			//		cur_pos[0] += horizontal_step;
			//	}
			//	else if (i == 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance / 2, view_distance, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] += horizontal_step;
			//	}
			//	else if (i >= delta - 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance / 2, view_distance, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] += horizontal_step;
			//	}
			//	else
			//	{
			//		focus_point = cur_pos + Eigen::Vector3d(0, view_distance, focus_z_corner_normal);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] += horizontal_step;
			//	}
			//}
			//delta = (ymax + view_distance - cur_pos.y()) / horizontal_step;
			//delta = delta > 4 ? delta : 4.1;
			//v_buildings[id_building].one_pass_trajectory_num += int(delta);
			//for (int i = 0; i < delta; ++i) {
			//	if (i == 0) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance, view_distance, focus_z_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] += horizontal_step;
			//	}
			//	else if (i == 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance, view_distance /2, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] += horizontal_step;
			//	}
			//	else if (i >= delta - 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance, -view_distance / 2, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] += horizontal_step;
			//	}
			//	else {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance, 0, focus_z_corner_normal);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] += horizontal_step;
			//	}
			//}
			//delta = (cur_pos.x() - (xmin - view_distance)) / horizontal_step;
			//delta = delta > 4 ? delta : 4.1;
			//v_buildings[id_building].one_pass_trajectory_num += int(delta);
			//for (int i = 0; i < delta; ++i) {
			//	if (i == 0) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance, -view_distance, focus_z_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] -= horizontal_step;
			//	}
			//	else if (i == 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(-view_distance /2, -view_distance, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] -= horizontal_step;
			//	}
			//	else if (i >= delta - 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance / 2, -view_distance, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] -= horizontal_step;
			//	}
			//	else {
			//		focus_point = cur_pos + Eigen::Vector3d(0, -view_distance, focus_z_corner_normal);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[0] -= horizontal_step;
			//	}
			//}
			//delta = (cur_pos.y() - (ymin - view_distance)) / horizontal_step;
			//delta = delta > 4 ? delta : 4.1;
			//v_buildings[id_building].one_pass_trajectory_num += int(delta);
			//for (int i = 0; i <  delta; ++i) {
			//	if (i == 0) {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance, -view_distance, focus_z_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] -= horizontal_step;
			//	}
			//	else if (i == 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance, -view_distance /2, focus_z_second_corner);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] -= horizontal_step;
			//	}
			//	else if (i >= delta - 1) {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance, view_distance /2, focus_z_second_corner);
			//		//item_trajectory.emplace_back(
			//		//	cur_pos, focus_point
			//		//);
			//		cur_pos[1] -= horizontal_step;
			//	}
			//	else {
			//		focus_point = cur_pos + Eigen::Vector3d(view_distance, 0, focus_z_corner_normal);
			//		item_trajectory.emplace_back(
			//			cur_pos, focus_point
			//		);
			//		cur_pos[1] -= horizontal_step;
			//	}
			//}
			v_buildings[id_building].one_pass_trajectory_num+=item_trajectory.size();
		}

		if(v_params["with_continuous_height_flag"].asBool() && v_params["double_flag"].asBool())
		{
			int num_views = item_trajectory.size();
			float height_max = zmax + z_up_bounds;
			float height_min = (zmax + z_up_bounds - z_down_bounds) / num_pass;

			float height_delta = height_max - height_min;
			float height_step = height_delta / num_views;

			int id = 0; 
			for (auto& item : item_trajectory)
			{
				item.pos_mesh.z() = std::max(height_max - id * height_step,z_down_bounds);
				++id;
			}
		}

		Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
		transform.translate(v_buildings[id_building].bounding_box_3d.box.center());
		transform.rotate(Eigen::AngleAxisd(v_buildings[id_building].bounding_box_3d.cv_box.angle/180.f*M_PI, Eigen::Vector3d::UnitZ()));
		transform.translate(-v_buildings[id_building].bounding_box_3d.box.center());
		for (int i=0;i< item_trajectory.size();++i)
		{
			item_trajectory[i].pos_mesh = transform * item_trajectory[i].pos_mesh;
			item_trajectory[i].focus_point = transform * item_trajectory[i].focus_point;
			item_trajectory[i].calculate_direction();
		}
		
		if(v_params["with_erosion_flag"].asBool()) // �Ȱ����flag����
			item_trajectory = find_short_cut(item_trajectory, v_height_map, safe_distance, v_buildings[id_building], v_tree);
		else
			item_trajectory = ensure_safe_trajectory(item_trajectory, v_height_map, safe_distance);

		if(v_params.isMember("z_down_bounds"))
		{
			for (int i = 0;i < item_trajectory.size();++i)
			{
				if (item_trajectory[i].pos_mesh.z() < v_params["z_down_bounds"].asFloat())
				{
					item_trajectory[i].pos_mesh.z() = v_params["z_down_bounds"].asFloat();
					item_trajectory[i].calculate_direction();
				}
			}
		}
		
		if (v_params["cluster_duplicate_flag"].asBool())
			cluster_duplicate(item_trajectory, v_params["vertical_overlap"].asFloat(), v_params["fov"].asFloat(),view_distance);

		v_buildings[id_building].trajectory = item_trajectory;
		total_trajectory.insert(total_trajectory.end(), item_trajectory.begin(), item_trajectory.end());
		v_buildings[id_building].is_changed = false;
	}
	return total_trajectory;
}

void generate_distance_map(const cv::Mat& v_map, cv::Mat& distance_map, const Eigen::Vector2i goal, Eigen::Vector2i now_point, int distance)
{
	if (now_point != goal)
	{
		if (now_point.x() < 0 || now_point.x() > v_map.rows - 1 || now_point.y() < 0 || now_point.y() > v_map.cols - 1)
			return;
		if (v_map.at<cv::int32_t>(now_point.x(), now_point.y()) == 0)
			return;
		if (distance_map.at<cv::int32_t>(now_point.x(), now_point.y()) != 0)
		{
			if (distance < distance_map.at<cv::int32_t>(now_point.x(), now_point.y()))
				distance_map.at<cv::int32_t>(now_point.x(), now_point.y()) = distance;
			else
				return;
		}
		else
			distance_map.at<cv::int32_t>(now_point.x(), now_point.y()) = distance;
	}
	else
		distance_map.at<cv::int32_t>(now_point.x(), now_point.y()) = 0;
	
	distance++;
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() + 1, now_point.y()), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x(), now_point.y() + 1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() - 1, now_point.y()), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x(), now_point.y() - 1), distance);
	//generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() + 1, now_point.y()+1), distance);
	//generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x()-1, now_point.y() + 1), distance);
	//generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() - 1, now_point.y()-1), distance);
	//generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x()+1, now_point.y() - 1), distance);
	
}

void update_obstacle_map(const cv::Mat& v_map, cv::Mat& distance_map, const Eigen::Vector2i goal, Eigen::Vector2i now_point, int distance)
{
	distance++;
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() + 1, now_point.y()), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x(), now_point.y() + 1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() - 1, now_point.y()), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x(), now_point.y() - 1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() + 1, now_point.y()+1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x()-1, now_point.y() + 1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x() - 1, now_point.y()-1), distance);
	generate_distance_map(v_map, distance_map, goal, Eigen::Vector2i(now_point.x()+1, now_point.y() - 1), distance);
}

void print_map(const cv::Mat& v_map)
{
	for (int i = 0; i < v_map.rows; i++)
	{
		for (int j = 0; j < v_map.cols; j++)
		{
			std::cout << int(v_map.at<cv::int32_t>(i, j)) << " ";
		}
		std::cout << std::endl;
	}
	return;
}

void explore(const cv::Mat& v_map, const cv::Mat& distance_map, const cv::Mat& obstacle_map, std::vector<Eigen::Vector2i>& trajectory, Eigen::Vector2i now_point, const Eigen::Vector2i& goal, cv::Mat& visited_map, bool& isFinished, float weight)
{
	trajectory.push_back(now_point);
	visited_map.at<cv::int32_t>(now_point.x(), now_point.y()) += 1;
	if (now_point == goal)
	{
		bool isCC = true;
		for (int i = 0; i < v_map.rows; i++)
		{
			for (int j = 0; j < v_map.cols; j++)
			{
				if (v_map.at<cv::int32_t>(i, j) != 0)
				{
					if (visited_map.at<cv::int32_t>(i, j) == 0)
					{
						isCC = false;
						break;
					}
				}
			}
			if (!isCC)
				break;
		}
		if (isCC)
		{
			isFinished = true;
			return;
		}
	}
	std::vector<Eigen::Vector2i> neighbors;
	neighbors.push_back(Eigen::Vector2i(now_point.x() + 1, now_point.y()));
	neighbors.push_back(Eigen::Vector2i(now_point.x(), now_point.y() + 1));
	neighbors.push_back(Eigen::Vector2i(now_point.x(), now_point.y() - 1));
	neighbors.push_back(Eigen::Vector2i(now_point.x() - 1, now_point.y()));

	//neighbors.push_back(Eigen::Vector2i(now_point.x()-1, now_point.y() - 1));
	//neighbors.push_back(Eigen::Vector2i(now_point.x()-1, now_point.y() + 1));
	//neighbors.push_back(Eigen::Vector2i(now_point.x()+1, now_point.y() - 1));
	//neighbors.push_back(Eigen::Vector2i(now_point.x()+1, now_point.y() + 1));
	// down right up left
	for (int i = 0; i < neighbors.size(); i++)
	{
		int max_num = -1;
		int max_id = -1;
		int now_id = 0;
		int temp_num;
		for (auto neighbor_point : neighbors)
		{
			if (!(neighbor_point.x() < 0 || neighbor_point.x() > v_map.rows - 1 || neighbor_point.y() < 0 || neighbor_point.y() > v_map.cols - 1))
			{

				temp_num = int(v_map.at<cv::int32_t>(neighbor_point.x(), neighbor_point.y()));
				if (temp_num != 0)
				{
					temp_num = int(visited_map.at<cv::int32_t>(neighbor_point.x(), neighbor_point.y()));
					if (temp_num == 0)
					{
						float distance = float(distance_map.at<cv::int32_t>(neighbor_point.x(), neighbor_point.y())) - weight * float(obstacle_map.at<cv::int32_t>(neighbor_point.x(), neighbor_point.y()));
						if (distance > max_num)
						{
							max_num = distance;
							max_id = now_id;
						}
					}
				}
			}
			now_id++;
		}
		if (max_id >= 0)
		{
			/*if (max_id >= 4)
			{
				if (int(visited_map.at<cv::uint8_t>(neighbors[max_id].x(), now_point.y())) == 0)
					trajectory.push_back(Eigen::Vector2i(neighbors[max_id].x(), now_point.y()));
				else
					trajectory.push_back(Eigen::Vector2i(now_point.x(), neighbors[max_id].y()));
			}*/
			//if (i != 0)
			//{
			//	trajectory.push_back(now_point);
			//}
			explore(v_map, distance_map, obstacle_map, trajectory, neighbors[max_id], goal, visited_map, isFinished, weight);
		}
		else
		{
			if (now_point == goal)
			{
				bool isCC = true;
				for (int i = 0; i < v_map.rows; i++)
				{
					for (int j = 0; j < v_map.cols; j++)
					{
						if (v_map.at<cv::int32_t>(i, j) != 0)

						{
							if (visited_map.at<cv::int32_t>(i, j) == 0)
							{
								isCC = false;
								break;
							}
						}
					}
					if (!isCC)
						break;
				}
				if (isCC)
				{
					isFinished = true;
					//trajectory.push_back(now_point);
					return;
				}
			}
			if (isFinished)
				return;
			else
			{
				bool isCC = true;
				int min_length = v_map.rows + v_map.cols;
				Eigen::Vector2i next_point;
				for (int i = 0; i < v_map.rows; i++)
				{
					for (int j = 0; j < v_map.cols; j++)
					{
						if (v_map.at<cv::int32_t>(i, j) != 0)
						{
							if (visited_map.at<cv::int32_t>(i, j) == 0)
							{
								isCC = false;
								int temp_distance = std::abs(now_point.x() - i) + std::abs(now_point.y() - j);
								if (temp_distance < min_length)
								{
									min_length = temp_distance;
									next_point = Eigen::Vector2i(i, j);
								}
							}
						}
					}
				}
				if (isCC)
				{
					isFinished = true;
					//trajectory.push_back(goal);
					return;
				}
				else
				{
					explore(v_map, distance_map, obstacle_map, trajectory, next_point, goal, visited_map, isFinished, weight);
					break;
				}	
			}
		}
	}
}

std::vector<Eigen::Vector2i> perform_ccpp(const cv::Mat& ccpp_map, const Eigen::Vector2i& v_start_point, const Eigen::Vector2i& v_goal, float weight = 1)
{
	cv::Mat v_map(ccpp_map.rows + 2, ccpp_map.cols + 2, CV_32SC1);
	for (int i = 0; i < v_map.rows; i++)
	{
		for (int j = 0; j < v_map.cols; j++)
		{
			if (i == 0 || j == 0 || i == v_map.rows - 1 || j == v_map.cols - 1)
			{
				v_map.at<cv::int32_t>(i, j) = 0;
			}
			else
			{
				v_map.at<cv::int32_t>(i, j) = ccpp_map.at<cv::uint8_t>(i - 1, j - 1);
			}
		}
	}
	std::vector<Eigen::Vector2i> trajectory;
	bool isAllBlack = false;
	Eigen::Vector2i goal(v_goal.y() + 1, v_goal.x() + 1);
	Eigen::Vector2i start_point(v_start_point.y() + 1, v_start_point.x() + 1);

	// Test
	//goal = start_point;

	int min_length = v_map.rows + v_map.cols;
	if (v_map.at<cv::int32_t>(start_point.x(),start_point.y()) == 0)
	{
		//trajectory.push_back(start_point);
		for (int i = 0; i < v_map.rows; i++)
		{
			for (int j = 0; j < v_map.cols; j++)
			{
				if (v_map.at<cv::int32_t>(i, j) != 0)
				{
					int temp_length = std::abs(v_start_point.y() - i + 1) + std::abs(v_start_point.x() - j + 1);
					if (temp_length < min_length)
					{
						min_length = temp_length;
						start_point = Eigen::Vector2i(i, j);
					} 
				}
			}
		}
		if (min_length == v_map.rows + v_map.cols)
		{
			trajectory.clear();
		}
	}
	
	cv::Mat distance_map(v_map.rows, v_map.cols, CV_32SC1, cv::Scalar(0));
	generate_distance_map(v_map, distance_map, goal, goal, 0);
	//std::cout << "distance_map" << std::endl;
	//print_map(distance_map);
	cv::Mat obstacle_map(v_map.rows, v_map.cols, CV_32SC1, cv::Scalar(0));
	for (int i = 0; i < v_map.rows; i++)
	{
		for (int j = 0; j < v_map.cols; j++)
		{
			if (v_map.at<cv::int32_t>(i, j) == 0)
			{
				update_obstacle_map(v_map, obstacle_map, Eigen::Vector2i(i, j), Eigen::Vector2i(i, j), 0);
			}
		}
	}
	//std::cout << "v_map" << std::endl;
	//print_map(v_map);
	//std::cout << "obstacle_map" << std::endl;
	//print_map(obstacle_map);

	Eigen::Vector2i now_point(start_point);
	cv::Mat visited_map(v_map.rows, v_map.cols, CV_32SC1, cv::Scalar(0));
	bool isFinished = false;
	explore(v_map, distance_map, obstacle_map, trajectory, start_point, goal, visited_map, isFinished, weight);
	//if (v_map.at<cv::int32_t>(goal.x(), goal.y()) == 0)
	//	trajectory.push_back(goal);

	cv::Mat sequence_map(v_map.rows, v_map.cols, CV_32SC1, cv::Scalar(0));
	for (int i = 0; i < trajectory.size(); i++)
	{
		sequence_map.at<cv::int32_t>(trajectory[i].x(), trajectory[i].y()) = i;
	}
	for (auto& trajectory_point : trajectory)
	{
		trajectory_point[0] -= 1;
		trajectory_point[1] -= 1;
	}
	//print_map(sequence_map);
	return trajectory;
}
