#include "metrics.h"

#include <boost/format.hpp>

std::vector<Eigen::Vector3d> sample_points_on_sphere(const int v_vertical_number)
{
	std::vector<Eigen::Vector3d> test_vecs;
	double theta_interval = M_PI / v_vertical_number;
	for (int i = 0; i < v_vertical_number; ++i)
	{
		double theta = i * theta_interval;
		int m = static_cast<int>(std::max(std::floor(v_vertical_number * std::sin(theta)), 1.));
		double fi_interval = 2 * M_PI / m;
		for (int j = 0; j < m; ++j)
		{
			double fi = fi_interval * j;
			test_vecs.emplace_back(std::sin(theta) * std::sin(fi), std::cos(theta), std::sin(theta) * std::cos(fi));
		}
	}
	return test_vecs;
}


std::vector<std::vector<bool>> compute_visibility(const std::vector<Viewpoint>& v_viewpoints,
	const SurfaceMesh& v_mesh, const PointSet3& point_set, const double v_fov_degree_h, const double v_fov_degree_v, const double v_dmax, Tree* v_tree)
{
	std::vector<std::vector<bool>> point_view_visibility;
	point_view_visibility.resize(point_set.size(), std::vector<bool>(v_viewpoints.size(), false));

	// Build AABB tree
	if (v_tree == nullptr)
	{
		v_tree = new Tree(CGAL::faces(v_mesh).first, CGAL::faces(v_mesh).second, v_mesh);
		v_tree->build();
	}

#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);

		//#pragma omp parallel for
		for (int j = 0; j < v_viewpoints.size(); j++)
		{
			if (intersectiontools::is_visible(v_viewpoints[j].pos_mesh, v_viewpoints[j].direction, cgaltools::cgal_point_2_eigen(point), *v_tree, v_fov_degree_h, v_fov_degree_v, v_dmax))
				point_view_visibility[i_point][j] = true;
		}
	}
	return point_view_visibility;
}

std::vector<std::vector<bool>> compute_visibility(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, Tree* v_tree)
{
	std::vector<std::vector<bool>> point_view_visibility;
	point_view_visibility.resize(point_set.size(), std::vector<bool>(v_viewpoints.size(), false));

	// Build AABB tree
	if (v_tree == nullptr)
	{
		v_tree = new Tree(CGAL::faces(v_mesh).first, CGAL::faces(v_mesh).second, v_mesh);
		v_tree->build();
	}

#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);

		//#pragma omp parallel for
		for (int j = 0; j < v_viewpoints.size(); j++)
		{

			if (intersectiontools::is_visible(
				v_intrinsic_matrix,
				MapConverter::get_pos_pack_from_direction_vector(v_viewpoints[j].pos_mesh, v_viewpoints[j].direction).camera_matrix, v_viewpoints[j].pos_mesh,
				cgaltools::cgal_point_2_eigen(point), *v_tree, v_dmax)
				) {
#pragma omp critical
					{
						point_view_visibility[i_point][j] = true;
					}
			}
		}
	}
	return point_view_visibility;
}

std::vector<std::vector<bool>> compute_visibility(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const RTCScene& v_embree_scene)
{
	std::vector<std::vector<bool>> point_view_visibility;
	point_view_visibility.resize(point_set.size(), std::vector<bool>(v_viewpoints.size(), false));

#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);

		//#pragma omp parallel for
		for (int j = 0; j < v_viewpoints.size(); j++)
		{

			if (intersectiontools::is_visible(
				v_intrinsic_matrix,
				MapConverter::get_pos_pack_from_direction_vector(v_viewpoints[j].pos_mesh, v_viewpoints[j].direction).camera_matrix, v_viewpoints[j].pos_mesh,
				cgaltools::cgal_point_2_eigen(point), v_embree_scene, v_dmax, v_mesh)
				) {
#pragma omp critical
					{
						point_view_visibility[i_point][j] = true;
					}
			}
		}
	}
	return point_view_visibility;
}



std::vector<std::vector<int>> compute_visibilityIndex(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, Tree* v_tree)
{
	std::vector<std::vector<int>> point_view_visibility(point_set.size());

	// Build AABB tree
	if (v_tree == nullptr)
	{
		v_tree = new Tree(CGAL::faces(v_mesh).first, CGAL::faces(v_mesh).second, v_mesh);
		v_tree->build();
	}

#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);

		for (int j = 0; j < v_viewpoints.size(); j++)
		{

			if (intersectiontools::is_visible(
				v_intrinsic_matrix,
				MapConverter::get_pos_pack_from_direction_vector(v_viewpoints[j].pos_mesh, v_viewpoints[j].direction).camera_matrix, v_viewpoints[j].pos_mesh,
				cgaltools::cgal_point_2_eigen(point), *v_tree, v_dmax)
				) {
#pragma omp critical
					{
						point_view_visibility.at(i_point).push_back(j);
					}
			}
		}
	}
	return point_view_visibility;
}

std::vector<std::vector<int>> compute_visibilityIndex(
	const std::vector<Viewpoint>& v_viewpoints, const SurfaceMesh& v_mesh,
	const PointSet3& point_set,
	const Eigen::Matrix3d& v_intrinsic_matrix,
	const double v_dmax, const RTCScene& v_embree_scene)
{
	std::vector<std::vector<int>> point_view_visibility(point_set.size());

#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);

		for (int j = 0; j < v_viewpoints.size(); j++)
		{

			if (intersectiontools::is_visible(
				v_intrinsic_matrix,
				MapConverter::get_pos_pack_from_direction_vector(v_viewpoints[j].pos_mesh, v_viewpoints[j].direction).camera_matrix, v_viewpoints[j].pos_mesh,
				cgaltools::cgal_point_2_eigen(point), v_embree_scene, v_dmax, v_mesh)
				) {
#pragma omp critical
					{
						point_view_visibility.at(i_point).push_back(j);
					}
			}
		}
	}
	return point_view_visibility;
}


std::array<double, 5> calculate_point_reconstructability(const Viewpoint& v_view1, const Viewpoint& v_view2,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double dmax)
{
	int k1 = 32;
	double alpha1 = 3.1415926f / 16;
	int k3 = 8;
	double alpha3 = 3.1415926f / 4;

	Eigen::Vector3d view_to_point1 = v_point - v_view1.pos_mesh;
	Eigen::Vector3d view_to_point2 = v_point - v_view2.pos_mesh;

	double alpha = std::acos(std::clamp(static_cast<double>(view_to_point1.normalized().dot(view_to_point2.normalized())), -1., 1.));
	double omega1 = 1. / (1 + std::exp(-k1 * (alpha - alpha1)));
	double omega2 = 1 - std::min(std::max(view_to_point1.norm(), view_to_point2.norm()) / dmax, 1.);
	double omega3 = 1. - 1. / (1 + std::exp(-k3 * (alpha - alpha3)));
	double Theta1 = (-view_to_point1).normalized().dot(v_normal);
	double Theta2 = (-view_to_point2).normalized().dot(v_normal);
	double cosTheta = std::min(Theta1, Theta2);
	double value = omega1 * omega2 * omega3 * cosTheta;
	return std::array<double, 5>{omega1, omega2, omega3, cosTheta, value > 0 ? value : 0};
}
double biviewsREC(const Eigen::Vector3d& v_view1, const Eigen::Vector3d& v_view2,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double dmax)
{

	const Eigen::Vector3d view_to_point1 = v_point - v_view1;
	const Eigen::Vector3d view_to_point2 = v_point - v_view2;

	const double alpha = std::acos(std::clamp(static_cast<double>(view_to_point1.normalized().dot(view_to_point2.normalized())), -1., 1.));
	const double omega2 = 1 - std::min(std::max(view_to_point1.norm(), view_to_point2.norm()) / dmax, 1.);
	const double cosTheta = std::min((-view_to_point1).normalized().dot(v_normal), (-view_to_point2).normalized().dot(v_normal));
	const double value = (1. / (1 + std::exp(-32 * (alpha - M_PI / 16.)))) * omega2 * (1. - 1. / (1 + std::exp(-8 * (alpha - M_PI / 4.)))) * cosTheta;
	return value > 0 ? value : 0;
}

std::vector<std::array<double, 5>> reconstructability_hueristic(std::vector<Viewpoint> trajectory,
	const PointSet3& point_set,
	const SurfaceMesh& v_mesh,
	std::vector<std::vector<bool>>& point_view_visibility,
	const double dmax, const double v_fov_degree_h, const double v_fov_degree_v,
	Tree* v_tree)
{
	if (!point_set.has_normal_map())
	{
		std::cout << "Point set do not have normals" << std::endl;
		throw;
	}
	std::vector<std::array<double, 5>> reconstructabilities(point_set.size());

	if (point_view_visibility.size() == 0)
	{
		point_view_visibility = compute_visibility(trajectory, v_mesh, point_set, v_fov_degree_h, v_fov_degree_v, dmax, v_tree);
	}
	else if (point_view_visibility.size() != point_set.size())
	{
		std::cout << "Point set size does not match visibility array size" << std::endl;
		throw;
	}

	// Build AABB tree
	if (v_tree == nullptr)
	{
		v_tree = new Tree(CGAL::faces(v_mesh).first, CGAL::faces(v_mesh).second, v_mesh);
		v_tree->build();
	}

	int log_step = 0;
	//#pragma omp parallel for
	for (int i_point = 0; i_point < point_set.size(); ++i_point)
	{
		const Point3& point = point_set.point(i_point);
		const Vector3& normal = point_set.normal(i_point) / std::sqrt(point_set.normal(i_point).squared_length());

		std::vector<std::array<double, 5>> point_recon;
#pragma omp parallel for
		for (int id_view1 = 0; id_view1 < trajectory.size(); id_view1++)
		{
			if (!point_view_visibility[i_point][id_view1])
				continue;
			for (int id_view2 = id_view1 + 1; id_view2 < trajectory.size(); id_view2++)
			{
				const Eigen::Vector3d point_eigen(point.x(), point.y(), point.z());
				Eigen::Vector3d normal_eigen(normal.x(), normal.y(), normal.z());
				if (!point_view_visibility[i_point][id_view2])
					continue;
#pragma omp critical
				{
					point_recon.push_back(calculate_point_reconstructability(trajectory[id_view1], trajectory[id_view2], point_eigen, normal_eigen, dmax));
				}
			}
		}


		//std::cout<<"Reconstructability Smith18 "<< std::endl;
		//  int log_step=0;
		//for (int id_view1 = 0; id_view1 < trajectory.size(); id_view1++)
		//{
		//	for (int id_view2 = id_view1+1; id_view2 < trajectory.size(); id_view2++)
		//		std::cout<<(boost::format("%3d, ")%std::get<4>(point_recon[log_step++])).str();
		//	std::cout<<std::endl;
		//}

		std::array<double, 5> total_recon = std::accumulate(point_recon.begin(), point_recon.end(), std::array<double, 5>{0., 0., 0., 0., 0.}, [](std::array<double, 5> sum, auto item)
			{
				sum[0] += std::get<0>(item);
				sum[1] += std::get<1>(item);
				sum[2] += std::get<2>(item);
				sum[3] += std::get<3>(item);
				sum[4] += std::get<4>(item);
				return sum;
			});

		//double num_view_pair = trajectory.size()* (trajectory.size()-1) / 2;
		double num_view_pair = 1;
		reconstructabilities[i_point] = std::array<double, 5>{
			total_recon[0] / num_view_pair,
				total_recon[1] / num_view_pair,
				total_recon[2] / num_view_pair,
				total_recon[3] / num_view_pair,
				(total_recon[4] > 0 ? total_recon[4] : 0) / num_view_pair};

#pragma omp critical
		{
			if (log_step != 0 && log_step % (point_set.size() / 10) == 0)
				LOG(INFO) << log_step << " / " << point_set.size();
			log_step += 1;
		}
	}

	return reconstructabilities;
}

double normal_distribution(const double v_x, const double v_exponential, double v_sigma)
{
	return std::exp(-std::pow(v_x, v_exponential) / 2 / v_sigma / v_sigma);
}

struct Default_lalala_parameters
{
	double alpha_weights_sigma1 = 5. / 90.;
	double alpha_weights_exponential1 = 2.;
	double alpha_weights_sigma2 = 30. / 90.;
	double alpha_weights_exponential2 = 2.;

	double gsd_weights_sigma = 1.5;
	double gsd_weights_exponential = 4.;

	double scale_weights_sigma = .25;
	double scale_weights_exponential = 2.;

	double angle_to_normal_weights_sigma = 30. / 90.;
	double angle_to_normal_weights_exponential = 2.;

	double distortion_weights_sigma = 1.;
	double distortion_weights_exponential = 2.;
};

std::vector<double> compute_reconstructability_lalala_item(const Viewpoint& v_view1, const Viewpoint& v_view2,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal, const double v_max_distance,
	const double v_fov_degree_h, const double v_fov_degree_v)
{
	Default_lalala_parameters default_lalala_parameters;
	// Base ability for triangulation and feature matching
	// Both in Furukawa et al.[2010] and Smith et al.[2018]
	Eigen::Vector3d view_to_point1 = v_point - v_view1.pos_mesh;
	Eigen::Vector3d view_to_point2 = v_point - v_view2.pos_mesh;
	double alpha_in_degree = std::acos(std::clamp(view_to_point1.normalized().dot(view_to_point2.normalized()), -1., 1.)) / M_PI * 180.;
	double base_score; // alpha ~ [0,180] -> base_score ~ [0,1]
	if (alpha_in_degree < 20.)
		base_score = normal_distribution(alpha_in_degree / 90., default_lalala_parameters.alpha_weights_exponential1, default_lalala_parameters.alpha_weights_sigma1); // Furukawaet al.[2010]
	else
		base_score = normal_distribution(alpha_in_degree / 90., default_lalala_parameters.alpha_weights_exponential2, default_lalala_parameters.alpha_weights_sigma2); // Furukawaet al.[2010] Modified the parameter

	double distance1 = view_to_point1.norm();
	double distance2 = view_to_point2.norm();
	// GSD
	// distance / max_distance = 1 -> GSD_score = 0.7f
	// distance / max_distance = 1.5 -> GSD_score = 0.15f
	// distance / max_distance = 2 -> GSD_score = 0.01f
	double GSD_score = normal_distribution(distance1 / v_max_distance, default_lalala_parameters.gsd_weights_exponential, default_lalala_parameters.gsd_weights_sigma) *
		normal_distribution(distance2 / v_max_distance, default_lalala_parameters.gsd_weights_exponential, default_lalala_parameters.gsd_weights_sigma);

	// Scale factor for feature matching
	// min(distance1,distance2) / max(distance1,distance2)
	// 0.2 -> scale_score = 0.9
	// 0.5 -> scale_score = 0.6
	// 0.75 -> scale_score = 0.3
	// 1 -> scale_score = 0.15
	double scale_score = normal_distribution(1 - std::min(distance1, distance2) / std::max(distance1, distance2),
		default_lalala_parameters.scale_weights_exponential, default_lalala_parameters.scale_weights_sigma);

	// Direction to point normal
	double angle_to_normal1 = std::acos(std::clamp(static_cast<double>(-(view_to_point1.normalized()).dot(v_normal.normalized())), -1., 1.)) / M_PI * 180.;
	double angle_to_normal2 = std::acos(std::clamp(static_cast<double>(-(view_to_point2.normalized()).dot(v_normal.normalized())), -1., 1.)) / M_PI * 180.;
	double normal_score = normal_distribution(angle_to_normal1 / 90.,
		default_lalala_parameters.angle_to_normal_weights_exponential, default_lalala_parameters.angle_to_normal_weights_sigma) * normal_distribution(angle_to_normal2 / 90.,
			default_lalala_parameters.angle_to_normal_weights_exponential, default_lalala_parameters.angle_to_normal_weights_sigma);

	// Direction of lens distortion
	// 0.1 (fov/2) -> disortion_score = 1
	// 0.5 (fov/2) -> disortion_score = 0.9
	// 0.75 (fov/2) -> disortion_score = 0.5
	// 0.9 (fov/2) -> disortion_score = 0.2
	double angle_to_view_direction1 = std::acos(std::clamp(static_cast<double>(view_to_point1.normalized().dot(v_view1.direction.normalized())), -1., 1.)) / (v_fov_degree_v / 180. * M_PI);
	double angle_to_view_direction2 = std::acos(std::clamp(static_cast<double>(view_to_point2.normalized().dot(v_view2.direction.normalized())), -1., 1.)) / (v_fov_degree_v / 180. * M_PI);
	double disortion_score = normal_distribution(angle_to_view_direction1,
		default_lalala_parameters.distortion_weights_exponential, default_lalala_parameters.distortion_weights_sigma) * normal_distribution(angle_to_view_direction2,
			default_lalala_parameters.distortion_weights_exponential, default_lalala_parameters.distortion_weights_sigma);

	double value = base_score * GSD_score * scale_score * normal_score * disortion_score;
	return std::vector<double>{base_score, GSD_score, scale_score, normal_score, disortion_score, value};
}

void find_min_max_path(std::vector<Eigen::MatrixXd>& v_distance_matrix, const int max_depth)
{
	const int num_views = v_distance_matrix[0].rows();
	for (int depth = 1; depth < max_depth; ++depth)
	{
		for (int i_start = 0; i_start < num_views - 1; ++i_start)
		{
			for (int i_target = i_start + 1; i_target < num_views; ++i_target)
			{
				for (int i_neighbour = 0; i_neighbour < num_views; i_neighbour++)
				{
					if (i_neighbour == i_start || i_neighbour == i_target)
						continue;
					v_distance_matrix[depth](i_start, i_target) = std::max(v_distance_matrix[depth](i_start, i_target), std::max(v_distance_matrix[depth - 1](i_start, i_target),
						std::min(v_distance_matrix[depth - 1](i_start, i_neighbour), v_distance_matrix[depth - 1](i_neighbour, i_target))));
					v_distance_matrix[depth](i_target, i_start) = v_distance_matrix[depth](i_start, i_target);
				}
			}
		}
	}
	return;
}

double compute_reconstructability_lalala(const std::vector<Viewpoint>& v_viewpoints, const Point3& v_point,
	const Vector3& v_normal, const double v_suggest_distance, const double v_fov_degree_h, const double v_fov_degree_v, const Eigen::Matrix3d& v_intrinsic_matrix,
	const int v_index, const int v_redundancy_density, const bool v_with_minmax, const bool v_log_txt)
{
	const Eigen::Vector3d point = cgaltools::cgal_point_2_eigen(v_point);
	double centre_theta = std::acos(std::clamp((double)v_normal.z(), -1., 1.)); // Notations in phisics, theta is responsible for z axis
	double centre_phi = std::atan2(v_normal.y(), v_normal.x());
	Eigen::Isometry3d rotation;
	rotation.setIdentity();
	rotation.rotate(Eigen::AngleAxisd(centre_phi, Eigen::Vector3d(0., 0., 1.)));
	rotation.rotate(Eigen::AngleAxisd(centre_theta, Eigen::Vector3d(0., 1., 0.)));
	double sqrt_2_2 = std::sqrt(2.) / 2;

	// Reduce redundancy
	std::vector<Viewpoint> viewpoints_after_reduce_redundancy;
	if (v_redundancy_density > 0)
	{
		std::vector<Eigen::Vector3d> supplementary_view = sample_points_on_sphere(v_redundancy_density);

		// Rotate the supplementary view
		for (int i = supplementary_view.size() - 1; i >= 0; --i)
		{
			if (supplementary_view[i].z() < 0)
				supplementary_view.erase(supplementary_view.begin() + i);
			supplementary_view[i] = rotation * supplementary_view[i];
		}
		std::vector<std::vector<int>> exist_distance(supplementary_view.size());

		// Calculate the nearest view for each supplementary view
#pragma omp parallel for
		for (int i_planned_view = 0; i_planned_view < v_viewpoints.size(); ++i_planned_view)
		{
			int nearest_supplementary_id = -1; double nearest_distance = -99999.;
			for (int i_supplementary_view = 0; i_supplementary_view < supplementary_view.size(); ++i_supplementary_view)
			{
				double distance = v_viewpoints[i_planned_view].direction.dot(-supplementary_view[i_supplementary_view]);
				if (distance > nearest_distance)
				{
					nearest_distance = distance;
					nearest_supplementary_id = i_supplementary_view;
				}
			}
#pragma omp critical
			{
				exist_distance[nearest_supplementary_id].push_back(i_planned_view);
			}
		}

		// Refine
		const double DISTANCE_STEP = v_suggest_distance / 2;
#pragma omp parallel for
		for (int i_supplementary_view = 0; i_supplementary_view < supplementary_view.size(); ++i_supplementary_view)
		{
			std::sort(exist_distance[i_supplementary_view].begin(), exist_distance[i_supplementary_view].end(),
				[&v_viewpoints](const int& item1, const int& item2) {return (v_viewpoints[item1].pos_mesh - v_viewpoints[item2].pos_mesh).norm(); });

			double min_distance = -DISTANCE_STEP;
			for (int i_planned_view = 0; i_planned_view < exist_distance[i_supplementary_view].size(); i_planned_view++)
			{
				if ((v_viewpoints[exist_distance[i_supplementary_view][i_planned_view]].pos_mesh - point).norm() > min_distance + DISTANCE_STEP)
				{
#pragma omp critical
					{
						viewpoints_after_reduce_redundancy.push_back(v_viewpoints[exist_distance[i_supplementary_view][i_planned_view]]);
					}
					min_distance = (v_viewpoints[exist_distance[i_supplementary_view][i_planned_view]].pos_mesh - point).norm();
				}
			}
		}
	}
	else
		viewpoints_after_reduce_redundancy = v_viewpoints;

	// Calculate base reconstructability 
	std::vector<std::vector<std::vector<double>>> reconstructability_result(viewpoints_after_reduce_redundancy.size(), std::vector<std::vector<double>>(viewpoints_after_reduce_redundancy.size()));
#pragma omp parallel for
	for (int i_view1 = 0; i_view1 < viewpoints_after_reduce_redundancy.size(); ++i_view1)
	{
		for (int i_view2 = i_view1 + 1; i_view2 < viewpoints_after_reduce_redundancy.size(); ++i_view2)
		{
			auto reconstructability_base = compute_reconstructability_lalala_item(
				viewpoints_after_reduce_redundancy[i_view1], viewpoints_after_reduce_redundancy[i_view2], cgaltools::cgal_point_2_eigen(v_point),
				cgaltools::cgal_vector_2_eigen(v_normal), v_suggest_distance, v_fov_degree_h, v_fov_degree_v);
			reconstructability_result[i_view1][i_view2] = reconstructability_base;
			reconstructability_result[i_view2][i_view1] = reconstructability_base;
		}
	}


	// Find minmax path
	double total_reconstructability = 0.;
	if (v_with_minmax)
	{
		const int MAX_DEPTH = 5;
		std::vector<Eigen::MatrixXd> reconstructability_matrix(MAX_DEPTH, Eigen::MatrixXd(viewpoints_after_reduce_redundancy.size(), viewpoints_after_reduce_redundancy.size()));
		for (auto& item : reconstructability_matrix)
			item.fill(0.);
		for (int i_view1 = 0; i_view1 < viewpoints_after_reduce_redundancy.size(); ++i_view1)
			for (int i_view2 = 0; i_view2 < viewpoints_after_reduce_redundancy.size(); ++i_view2)
				if (i_view1 != i_view2)
					reconstructability_matrix[0](i_view1, i_view2) = reconstructability_result[i_view1][i_view2][5];
		find_min_max_path(reconstructability_matrix, MAX_DEPTH);
		for (int log_depth = 0; log_depth < MAX_DEPTH; ++log_depth)
		{
			//std::cout<<"Depth "<< log_depth<<std::endl;
			for (int i_start = 0; i_start < viewpoints_after_reduce_redundancy.size(); ++i_start)
			{
				for (int i_target = i_start + 1; i_target < viewpoints_after_reduce_redundancy.size(); ++i_target)
				{
					//std::cout<<(boost::format("%3d, ") % reconstructability_matrix[log_depth](i_start,i_target)).str();
					//total_reconstructability+=reconstructability_matrix[log_depth](i_start,i_target);
				}
				//std::cout<<std::endl;
			}
		}
		for (int i_start = 0; i_start < viewpoints_after_reduce_redundancy.size(); ++i_start)
			for (int i_target = i_start + 1; i_target < viewpoints_after_reduce_redundancy.size(); ++i_target)
				total_reconstructability += reconstructability_matrix[MAX_DEPTH - 1](i_start, i_target);
	}
	else
	{
		for (int i_start = 0; i_start < viewpoints_after_reduce_redundancy.size(); ++i_start)
			for (int i_target = i_start + 1; i_target < viewpoints_after_reduce_redundancy.size(); ++i_target)
				total_reconstructability += reconstructability_result[i_start][i_target][5];
	}

	// Log
	if (v_log_txt) {
		int resolution_x = static_cast<int>(v_intrinsic_matrix(0, 2) * 2);
		int resolution_y = static_cast<int>(v_intrinsic_matrix(1, 2) * 2);

		std::string out_str = "";
		out_str += std::to_string(viewpoints_after_reduce_redundancy.size()) + "\n";
		out_str += std::to_string(total_reconstructability) + "\n";
		for (int i_view1 = 0; i_view1 < viewpoints_after_reduce_redundancy.size(); ++i_view1)
		{
			auto pos_pack = MapConverter::get_pos_pack_from_direction_vector(viewpoints_after_reduce_redundancy[i_view1].pos_mesh, viewpoints_after_reduce_redundancy[i_view1].direction);
			auto pixel_position = v_intrinsic_matrix * pos_pack.camera_matrix * cgaltools::cgal_point_2_eigen(v_point);
			pixel_position /= pixel_position.z();

			Eigen::Vector3d view_to_point1 = cgaltools::cgal_point_2_eigen(v_point) - viewpoints_after_reduce_redundancy[i_view1].pos_mesh;
			double angle_to_normal1 = std::acos(std::clamp(static_cast<double>(-(view_to_point1.normalized()).dot(cgaltools::cgal_vector_2_eigen(v_normal).normalized())), -1., 1.)) / M_PI * 180.;
			double angle_to_view_direction1 = std::acos(std::clamp(static_cast<double>(view_to_point1.normalized().dot(viewpoints_after_reduce_redundancy[i_view1].direction.normalized())), -1., 1.));
			out_str += (boost::format("%s,%d,%d,%d,%d,%d,%d,%d,%d\n")
				% viewpoints_after_reduce_redundancy[i_view1].img_name
				% view_to_point1.x() % view_to_point1.y() % view_to_point1.z()
				% (view_to_point1.norm() / v_suggest_distance)
				% (angle_to_normal1 / 90.)
				% (angle_to_view_direction1 / (v_fov_degree_v / 180. * M_PI))
				% (pixel_position.x() / resolution_x) % (pixel_position.y() / resolution_y)
				).str();
		}
		for (int i_view1 = 0; i_view1 < viewpoints_after_reduce_redundancy.size(); ++i_view1)
		{
			for (int i_view2 = i_view1 + 1; i_view2 < viewpoints_after_reduce_redundancy.size(); ++i_view2)
			{
				Eigen::Vector3d view_to_point1 = cgaltools::cgal_point_2_eigen(v_point) - viewpoints_after_reduce_redundancy[i_view1].pos_mesh;
				Eigen::Vector3d view_to_point2 = cgaltools::cgal_point_2_eigen(v_point) - viewpoints_after_reduce_redundancy[i_view2].pos_mesh;
				double alpha = std::acos(std::clamp(static_cast<double>(view_to_point1.normalized().dot(view_to_point2.normalized())), -1., 1.)) / M_PI * 180.;
				out_str += (boost::format("%d,%d\n") %
					(alpha / 90.) %
					(1 - std::min(view_to_point1.norm(), view_to_point2.norm()) / std::max(view_to_point1.norm(), view_to_point2.norm()))
					).str();
			}
		}
		std::ofstream log_file("log/" + std::to_string(v_index) + ".txt", std::ofstream::binary | std::ios::out);
		log_file << out_str;
		log_file.close();
	}
	return total_reconstructability;
}

std::vector<double> compute_reconstructability_point_set_lalala(const std::vector<Viewpoint>& v_viewpoints,
	const PointSet3& v_point_set, const std::vector<std::vector<bool>>& point_view_visibility,
	const double v_max_distance, const double v_fov_degree_h, const double v_fov_degree_v, const Eigen::Matrix3d& v_intrinsic_matrix,
	const int v_redundancy_density, const bool v_with_minmax, const bool v_log_txt)
{
	std::vector<double> reconstructability(v_point_set.size(), 0.);
	int log_step = 0;
#pragma omp parallel for schedule(dynamic,1)
	for (int i_point = 0; i_point < v_point_set.size(); ++i_point)
	{
		std::vector<Viewpoint> viewpoints_visible_to_current_point;
		for (int i_view = 0; i_view < v_viewpoints.size(); ++i_view)
		{
			if (point_view_visibility[i_point][i_view])
				viewpoints_visible_to_current_point.push_back(v_viewpoints[i_view]);
		}
		if (viewpoints_visible_to_current_point.size() <= 1)
			reconstructability[i_point] = 0.;
		else
			reconstructability[i_point] = compute_reconstructability_lalala(viewpoints_visible_to_current_point, v_point_set.point(i_point), v_point_set.normal(i_point),
				v_max_distance, v_fov_degree_h, v_fov_degree_v, v_intrinsic_matrix, i_point, v_redundancy_density, v_with_minmax, v_log_txt);
#pragma omp critical
		{
			if (log_step != 0 && log_step % (v_point_set.size() / 10) == 0)
				LOG(INFO) << log_step << " / " << v_point_set.size();
			log_step += 1;
		}
	}
	return reconstructability;
}
