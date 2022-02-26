#include "initialize.h"

double evaluationOfViews(const Vector3& normal, const std::vector<Vector3>& exsitingViews, const Vector3& checkView)
{
	if (exsitingViews.size() == 1)
		return normal * checkView;
	const int harmonicValue = static_cast<int>(exsitingViews.size()) - 1;
	const double coefficientN = normal * checkView;
	double coefficientV = .0;
	for (const auto& i : exsitingViews)
	{
		coefficientV += i * checkView;
	}
	return coefficientN - 0.7 * (coefficientV / harmonicValue);
}

std::pair<std::vector<Viewpoint>, PointSet3> initialize_viewpoints(PointSet3& v_points, const SurfaceMesh& v_mesh,
	const RTCScene& v_scene, const modeltools::Height_map& v_height_map, const Eigen::Matrix3d& v_intrinsic_matrix, const double v_max_distance,
	const double v_view_distance, int v_max_iter, int v_init_num_per_point)
{
	std::vector<Viewpoint> o_trajectories;
	PointSet3 points_failed_to_initialized;
#pragma omp parallel for
	for (int i = 0; i < v_points.size(); ++i)
	{
		Point3 base, survey;
		Vector3 normal, direction;

		base = v_points.point(i);
		normal = v_points.normal(i);

		int viewnum = v_init_num_per_point;
		Vector3 defaultVec = Vector3(0, 0, 0);
		std::vector<Vector3> views;
		views.push_back(defaultVec);
		std::vector<Viewpoint> vs;

		bool first = true;

		std::mt19937 mt(0);
		std::normal_distribution<double> dis1(0, 1);
		std::uniform_real_distribution<double> dis_theta(-M_PI_2, M_PI_2);
		std::uniform_real_distribution<double> dis_phi(0, M_PI * 2);
		std::uniform_real_distribution<double> dis_distance(v_view_distance / 2, v_view_distance / 2 * 3);

		while (viewnum > 0)
		{

			double disItem;
			Vector3 bestCandidate;
			double theta = std::acos(normal.z()); // Notations in phisics
			double phi = std::atan2(normal.y(), normal.x());

			double view_distance_item;

			if (first)
			{
				direction = normal;
				survey = Point3(base + v_view_distance * direction);
			}
			else
			{
				double new_phi = phi + dis_phi(mt);
				double new_theta = dis_theta(mt) + theta;
				view_distance_item = dis_distance(mt);

				Vector3 new_normal(
					std::sin(new_theta) * std::cos(new_phi), std::sin(new_theta) * std::sin(new_phi),
					std::cos(new_theta));
				new_normal /= std::sqrt(new_normal.squared_length());
				survey = Point3(base + view_distance_item * new_normal);
				direction = new_normal;
			}

			bool initial = true;
			int threshold = v_max_iter;
			double maxeva = .0;

			while (threshold >= 0)
			{
				threshold -= 1;

				// Find collision
				//bool visiable = is_visible(
				//    intrinsicMatrix,
				//    MapConverter::get_pos_pack_from_direction_vector(
				//        cgal_point_2_eigen(survey), cgal_vector_2_eigen(-direction))
				//    .camera_matrix,
				//    cgal_point_2_eigen(survey),
				//    cgal_point_2_eigen(base), tree, maxIterTimes);
				bool visiable = intersectiontools::is_visible(
					v_intrinsic_matrix,
					MapConverter::get_pos_pack_from_direction_vector(
						cgaltools::cgal_point_2_eigen(survey), cgaltools::cgal_vector_2_eigen(-direction))
					.camera_matrix,
					cgaltools::cgal_point_2_eigen(survey),
					cgaltools::cgal_point_2_eigen(base), v_scene, 80, v_mesh);

				// No intersection and do not contain up view
				if (direction.z() >= 0 && visiable && v_height_map.is_safe(cgaltools::cgal_point_2_eigen(survey)) && survey.z() > 10.)
				{
					if (threshold == v_max_iter - 1 && first)
					{
						bestCandidate = direction;
						disItem = v_view_distance;
						initial = false;
						break;
					}
					if (!initial)
					{
						double cureva = evaluationOfViews(normal, views, direction);
						if (cureva > maxeva)
						{
							bestCandidate = direction;
							disItem = view_distance_item;
							maxeva = cureva;
						}
					}
					else
					{
						bestCandidate = direction;
						disItem = view_distance_item;
						maxeva = evaluationOfViews(normal, views, direction);
						initial = false;
					}
				}

				// Rotate it
				double new_phi = phi + dis_phi(mt);
				double new_theta = dis_theta(mt) + theta;
				view_distance_item = dis_distance(mt);

				Vector3 new_normal(
					std::sin(new_theta) * std::cos(new_phi), std::sin(new_theta) * std::sin(new_phi),
					std::cos(new_theta));
				new_normal /= std::sqrt(new_normal.squared_length());

				survey = Point3(base + view_distance_item * new_normal);

				direction = new_normal;
			} // close once view generation

			if (!initial)
			{
				Viewpoint vp;
				survey = Point3(base + disItem * bestCandidate);
				vp.pos_mesh = Eigen::Vector3d(survey.x(), survey.y(), survey.z());
				vp.direction = Eigen::Vector3d(-bestCandidate.x(), -bestCandidate.y(), -bestCandidate.z()).normalized();
				views.push_back(bestCandidate);
				vs.push_back(vp);
			}
			viewnum--;
			if (viewnum == v_init_num_per_point - 1)
			{
				first = false;
			}
		} // close while viewnum

		if (views.size() != 1)
		{
#pragma omp critical
			{
				for (const auto& v : vs)
				{
					o_trajectories.push_back(v);
				}
				// *********************************************************************
				/*PointSet3 p;
                p.insert(points.point(i));
                one2One.at(i) = { p,vs };*/
				// *********************************************************************
			}
		}
		else
		{
#pragma omp critical
			{
				points_failed_to_initialized.insert(v_points.point(i));
				v_points.remove(i);
				// *********************************************************************
				/*PointSet3 p;
                p.insert(Point3(0,0,0));
                one2One.at(i) = { p,vs };*/
				// *********************************************************************
			}
		}
	}

	LOG(INFO) << boost::format("Sucssesfully generate %d viewpoints for %d/%d points") % o_trajectories.size() % v_points.size() % (v_points.size() + points_failed_to_initialized.size());

	return std::make_pair(o_trajectories,points_failed_to_initialized);
}
