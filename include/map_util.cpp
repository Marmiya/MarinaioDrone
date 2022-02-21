/*
 * Convert coordinates in airsim, unreal, mesh, image
 */
#include "map_util.h"


MapConverter::MapConverter() {

}

void MapConverter::initDroneStart(const Eigen::Vector3d& vPos) {
	mDroneStart = vPos;
	initDroneDone = true;
}

Eigen::Vector3d MapConverter::convertUnrealToAirsim(const Eigen::Vector3d& vWorldPos) const {
	if (!initDroneDone)
		throw "Init is not done";
	Eigen::Vector3d result;
	result[0] = (vWorldPos[0] - mDroneStart[0]) / 100;
	result[1] = (vWorldPos[1] - mDroneStart[1]) / 100;
	result[2] = (-vWorldPos[2] + mDroneStart[2]) / 100;
	return result;
}

Eigen::Vector3d MapConverter::convertAirsimToMesh(const Eigen::Vector3d& vAirsimPos) const {
	if (!initDroneDone)
		throw "Init is not done";
	Eigen::Vector3d result;
	
	result[0] = vAirsimPos.x() * 100 + mDroneStart[0];
	result[1] = vAirsimPos.y() * 100 + mDroneStart[1];
	result[2] = -(vAirsimPos.z() * 100 - mDroneStart[2]);
	return convertUnrealToMesh(result);
}

Eigen::Vector3d MapConverter::convertUnrealToMesh(const Eigen::Vector3d& vWorldPos) const {
	if (!initDroneDone)
		throw "Init is not done";
	Eigen::Vector3d result;
	result[0] = (vWorldPos[0] / 100);
	result[1] = -(vWorldPos[1] / 100);
	result[2] = vWorldPos[2] / 100;
	return result;
}

Eigen::Vector3d MapConverter::convertMeshToUnreal(const Eigen::Vector3d& vMeshPos) const {
	if (!initDroneDone)
		throw "Init is not done";
	Eigen::Vector3d result;
	result[0] = (vMeshPos[0] * 100);
	result[1] = -(vMeshPos[1] * 100);
	result[2] = vMeshPos[2] * 100;
	return result;
}

Eigen::Matrix3d MapConverter::convert_yaw_pitch_to_matrix_mesh(const double yaw,const double pitch)
{
	Eigen::Matrix3d result = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(
		pitch, Eigen::Vector3d::UnitX())).toRotationMatrix();
	return result;
}

Eigen::Isometry3d MapConverter::get_camera_matrix(const double yaw, const double pitch,const Eigen::Vector3d& v_pos) {

	Eigen::Isometry3d result = Eigen::Isometry3d::Identity();

	result.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	result.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));

	result.rotate(Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()));
	result.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()));

	result.pretranslate(v_pos);

	return result.inverse();
}

Eigen::Vector3d MapConverter::convert_yaw_pitch_to_direction_vector(const double yaw,const double pitch)
{
	Eigen::Vector3d direction = Eigen::Vector3d(1, 0, 0);
	direction = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY())).toRotationMatrix() * direction;
	direction.normalize();
	return direction;
}

Pos_Pack MapConverter::get_pos_pack_from_direction_vector(const Eigen::Vector3d& v_pos_mesh,
	const Eigen::Vector3d& v_direction)
{
	Pos_Pack pos_pack;
	pos_pack.yaw = std::atan2(v_direction.y(),v_direction.x());
	pos_pack.pitch = -std::asin(v_direction.z());
	pos_pack.pos_mesh = v_pos_mesh;
	//pos_pack.pos_airsim = convertMeshToUnreal(convertUnrealToAirsim(v_pos_mesh));
	pos_pack.camera_matrix = get_camera_matrix(pos_pack.yaw, pos_pack.pitch, pos_pack.pos_mesh);
	pos_pack.direction = Eigen::Vector3d(
		std::cos(pos_pack.pitch)*std::cos(pos_pack.yaw),
		std::cos(pos_pack.pitch)*std::sin(pos_pack.yaw),
		std::sin(-pos_pack.pitch)).normalized();
	
	return pos_pack;
}

Pos_Pack MapConverter::get_pos_pack_from_unreal(const Eigen::Vector3d& v_pos_unreal,double v_unreal_yaw,double v_unreal_pitch)
{
	Pos_Pack pos_pack;
	pos_pack.yaw = -v_unreal_yaw;
	pos_pack.pitch = v_unreal_pitch;
	pos_pack.pos_mesh = convertUnrealToMesh(v_pos_unreal);
	pos_pack.pos_airsim = convertUnrealToAirsim(v_pos_unreal);
	pos_pack.camera_matrix = get_camera_matrix(pos_pack.yaw, pos_pack.pitch, pos_pack.pos_mesh);
	pos_pack.direction = Eigen::Vector3d(
		std::cos(pos_pack.pitch)*std::cos(pos_pack.yaw),
		std::cos(pos_pack.pitch)*std::sin(pos_pack.yaw),
		std::sin(-pos_pack.pitch)).normalized();
	
	return pos_pack;
}

Pos_Pack MapConverter::get_pos_pack_from_mesh(const Eigen::Vector3d& v_pos_mesh, double v_mesh_yaw, double v_mesh_pitch) {
	Pos_Pack pos_pack;
	pos_pack.yaw = v_mesh_yaw;
	pos_pack.pitch = v_mesh_pitch;
	pos_pack.pos_mesh = v_pos_mesh;
	pos_pack.pos_airsim = convertUnrealToAirsim(convertMeshToUnreal(v_pos_mesh));
	pos_pack.camera_matrix = get_camera_matrix(pos_pack.yaw, pos_pack.pitch, pos_pack.pos_mesh);
	pos_pack.direction = Eigen::Vector3d(
		std::cos(pos_pack.pitch) * std::cos(pos_pack.yaw),
		std::cos(pos_pack.pitch) * std::sin(pos_pack.yaw),
		std::sin(-pos_pack.pitch)).normalized();

	return pos_pack;
}