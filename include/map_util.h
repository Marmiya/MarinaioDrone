#pragma once
/*
 * Convert coordinates in airsim, unreal, mesh, image
 */
#include "common_util.h"

#include <Eigen/Dense>

struct Pos_Pack
{
	Eigen::Vector3d pos_mesh;
	Eigen::Vector3d pos_airsim;
	Eigen::Vector3d direction;
	Eigen::Isometry3d camera_matrix;

	double yaw, pitch;
};

class MapConverter {
public:
    MapConverter();

    Eigen::Vector3d mDroneStart;

    double mImageStartX;
    double mImageStartY;

    bool initDroneDone = false;
    bool initImageDone = false;

    void initDroneStart(const Eigen::Vector3d& vPos);

    Eigen::Vector3d convertUnrealToAirsim(const Eigen::Vector3d& vWorldPos) const;

    Eigen::Vector3d convertUnrealToMesh(const Eigen::Vector3d& vWorldPos) const;

    Eigen::Vector3d convertMeshToUnreal(const Eigen::Vector3d& vMeshPos) const;

    Eigen::Vector3d convertAirsimToMesh(const Eigen::Vector3d& vAirsimPos) const;
	
    Eigen::Matrix3d convert_yaw_pitch_to_matrix_mesh(const double yaw, const double pitch);

    static Eigen::Isometry3d get_camera_matrix(const double yaw, const double pitch, const Eigen::Vector3d& v_pos);

    static Eigen::Vector3d convert_yaw_pitch_to_direction_vector(const double yaw, const double pitch);

    static Pos_Pack get_pos_pack_from_direction_vector(const Eigen::Vector3d& v_pos_mesh, const Eigen::Vector3d& v_direction);
    Pos_Pack get_pos_pack_from_unreal(const Eigen::Vector3d& v_pos_unreal, double yaw, double pitch);
    Pos_Pack get_pos_pack_from_mesh(const Eigen::Vector3d& v_pos_mesh, double v_mesh_yaw, double v_mesh_pitch);

};