#pragma once

#include <corecrt_math_defines.h>
#include <random>
#include "map_util.h"
#include "metrics.h"


double
evaluationOfViews(
    const Vector3& normal, const std::vector<Vector3>& exsitingViews, const Vector3& checkView
);

// Generation of initial views.
std::pair<std::vector<Viewpoint>, PointSet3>
initialize_viewpoints(
    PointSet3& v_points, const SurfaceMesh& v_mesh, const RTCScene& v_scene,
    const modeltools::Height_map& v_height_map, const Eigen::Matrix3d& v_intrinsic_matrix,
    const double v_max_distance, const double v_view_distance, int v_max_iter, int v_init_num_per_point
);
