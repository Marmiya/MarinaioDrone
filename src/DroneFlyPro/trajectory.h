#pragma once

#include <boost/format.hpp>
#include <Eigen/Core>


namespace traj {

	struct Viewpoint
	{
		Eigen::Vector3f pos_mesh;
		Eigen::Vector3f direction;
		Eigen::Vector3f focus_point;
		std::string img_name;

		Viewpoint() = default;
		Viewpoint(const Eigen::Vector3f& v_pos_mesh, const Eigen::Vector3f& v_direction) : pos_mesh(v_pos_mesh), direction(v_direction) {}

		template <typename Archive>
		void serialize(Archive& ar, const unsigned int version)
		{
			ar& pos_mesh;
			ar& direction;
			ar& focus_point;
		}
	};


	class Trajectory {
	public:
		// elements:
		std::vector<Viewpoint> traj;
		std::vector<Viewpoint> keyTraj;

		// functions:
		Trajectory() = default;
	};
}
