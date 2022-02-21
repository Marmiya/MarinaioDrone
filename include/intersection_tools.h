#ifndef INTERSECTION_TOOLS_H
#define INTERSECTION_TOOLS_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <algorithm>

#include "cgal_tools.h"
#include "embree3/rtcore.h"

namespace intersectiontools {
    // @brief: Get the depth map by building a bvh tree and query the distance and instance label of each mesh
    // @notice: Distance is perspective distance, not planar distance!
    // @param:
    // @ret: `{ img_distance,img_distance_planar, img_distance_perspective} (0 represent the background, object id start by 1)`
    std::tuple<cv::Mat, cv::Mat, cv::Mat> get_depth_map_through_meshes(const std::vector<SurfaceMesh>& v_meshes,
        const int v_width, const int v_height,
        const Eigen::Matrix3d& v_intrinsic);

    PointSet3 remove_points_inside(const SurfaceMesh& v_mesh, const std::vector<Point3>& v_points);

	void remove_points_inside(const SurfaceMesh& v_mesh, PointSet3& v_points);

    bool is_visible(
        const Eigen::Vector3d& v_view_pos, const Eigen::Vector3d& v_view_direction, const Eigen::Vector3d& v_point_pos, 
        const Tree& v_tree, 
        const float fov_in_degree_h, const float fov_in_degree_v, float max_distance = -1.f
    );

    bool is_visible(
        const Eigen::Matrix3d& v_intrinsic_matrix, const Eigen::Isometry3d& v_camera_matrix, 
        const Eigen::Vector3d& v_view_pos, const Eigen::Vector3d& v_point_pos,
        const Tree& v_tree, float max_distance
    );

    RTCScene generate_embree_scene(const SurfaceMesh& v_mesh);

    // Use Embree to accelerate intersection test, not accurate but fast
    bool is_visible(
        const Eigen::Matrix3d& v_intrinsic_matrix, const Eigen::Isometry3d& v_camera_matrix,
        const Eigen::Vector3d& v_view_pos, const Eigen::Vector3d& v_point_pos,
        const RTCScene& v_scene, double max_distance, const SurfaceMesh& v_mesh);

    // Treat frustum as a cone, not accurate but fast
    // Use Embree to accelerate intersection test, not accurate but fast
    bool is_visible(
        const Eigen::Vector3d& v_view_pos, const Eigen::Vector3d& v_view_direction, const double v_fov_degree, const Eigen::Vector3d& v_point_pos,
        const RTCScene& v_scene, double max_distance, const SurfaceMesh& v_mesh);

}
#endif // INTERSECTION_TOOLS_H