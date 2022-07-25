#pragma once
#include <boost/format.hpp>
#include <cuda_runtime.h>
#include "common_util.h"
#include "map_util.h"

struct Viewpoint
{
	Eigen::Vector3d pos_mesh;
	Eigen::Vector3d direction;
	Eigen::Vector3d focus_point;
	std::string img_name;

	Viewpoint() = default;
	Viewpoint(const Eigen::Vector3d& v_pos_mesh, const Eigen::Vector3d& v_direction) :
		pos_mesh(v_pos_mesh), direction(v_direction) {}

	template <typename Archive>
	void serialize(Archive& ar, const unsigned int version)
	{
		ar& pos_mesh;
		ar& direction;
		ar& focus_point;
	}
	double3 getCudaPos() const
	{
		return make_double3(pos_mesh.x(), pos_mesh.y(), pos_mesh.z());
	}
	double3 getCudaDirection() const
	{
		return make_double3(direction.x(), direction.y(), direction.z());
	}
};

struct RouteViewpoint
{
	Eigen::Vector3d takeoff_station;
	std::vector<Viewpoint> viewpoints;
};