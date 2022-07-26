#include <iostream>
#include <list>
#include <boost/filesystem.hpp>
#include <argparse/argparse.hpp>
#include <json/reader.h> 
#include <glog/logging.h>
#include <filesystem>

#include "dronescan.h"
#include "metrics.h"
#include "initialize.h"
#include "SmithOptimized.h"
#include "trajectory.h"

int main(int argc, char** argv)
{
	// Read configuration.

	std::cout << "Read config " << argv[2] << std::endl;
	Json::Value args;
	{
		argparse::ArgumentParser program("Evaluate reconstructability");
		program.add_argument("--config_file").required();
		try
		{
			program.parse_args(argc, argv);
			const std::string config_file = program.get<std::string>("--config_file");

			std::ifstream in(config_file);
			if (!in.is_open())
			{
				LOG(ERROR) << "Error opening file" << config_file << std::endl;
				return 0;
			}
			Json::Reader json_reader;
			if (!json_reader.parse(in, args))
			{
				LOG(ERROR) << "Error parse config file" << config_file << std::endl;
				return 0;
			}
			in.close();
		}
		catch (const std::runtime_error& err)
		{
			std::cout << err.what() << std::endl;
			std::cout << program;
			exit(0);
		}
	}

	google::InitGoogleLogging(argv[0]);
	google::SetStderrLogging(google::GLOG_INFO);

	int STAGE = args["stage"].asInt();
	std::string logPath = args["logPath"].asString();
	auto logt = comutil::timeToString(std::chrono::system_clock::now());

	logPath += logt + " STAGE " + std::to_string(STAGE);
	fs::path lop(logPath);
	logPath += "/";
	fs::create_directory(lop);

	std::cout << "Stage: " << STAGE << "\n"
		<< "Log Path: " << logPath << "\n";

	// Mesh of the environment.
	SurfaceMesh mesh;
	mesh = modeltools::read_model(args["model"].asString());
	auto embree_scene = intersectiontools::generate_embree_scene(mesh);

	// AABB tree of this mesh.
	Tree tree(CGAL::faces(mesh).begin(), CGAL::faces(mesh).end(), mesh);
	tree.build();

	// Some Global variables.
	// -----------------------------------------------------------------------------------------------

	// The initial distance between view point and sample point.
	double viewDis = 15;
	// The max distance the drone's camera can watch.
	double maxViewDis = 80;
	double maxAngle = 15;
	// FOV.
	constexpr double fov_h = 62.;
	constexpr double fov_v = 42.;
	// Resolution.
	double resolution_h = 6000;
	double resolution_v = 4000;
	// Camera intrinsic matrix.
	Eigen::Matrix3d intrinsic_matrix;
	intrinsic_matrix <<
		resolution_h / 2 / std::tan(fov_h / 180 * M_PI / 2), 0, 2984.7599535479562,
		0, resolution_v / 2 / std::tan(fov_v / 180 * M_PI / 2), 2026.0429677971831,
		0, 0, 1;

	// The max loop times when generating initial view.
	constexpr int MAX_ITER_TIMES = 100;
	const int initialViewsAmount = args["initialViewNum"].asInt();

	if (STAGE == 0)
	{
		// The point set which is processed in the workflow below.
		PointSet3 points;
		CGAL::IO::read_point_set(args["points"].asString(), points);
		LOG(INFO) << boost::format("Totally %d points") % points.number_of_points();
		// Default color is white.
		auto colorMap = points.add_property_map("color", Eigen::Vector3d(1., 1., 1.)).first;

		// The original pointset read from .ply file.
		PointSet3 originalPts;
		auto colorOMap = points.add_property_map("color", Eigen::Vector3d(0., 1., 0.)).first;
		originalPts = points;

		// The trajectory which is processed in the workflow below.
		std::vector<Viewpoint> trajectory;
		// The final view set.
		std::vector<Viewpoint> finalAns;

		// Size of pointset and viewset.
		int pointsSize = static_cast<int>(points.size());
		int viewSize = static_cast<int>(trajectory.size());

		// Some variable for debugging.
		// *******************************************************************************************
		std::vector<std::pair<PointSet3, std::vector<Viewpoint>>> one2One(points.size());
		// *******************************************************************************************

		// Generation of initial views.
#pragma omp parallel for
		for (int i = 0; i < pointsSize; ++i)
		{
			Point3 base, survey;
			Vector3 normal, direction;

			base = points.point(i);
			normal = points.normal(i);

			int viewnum = initialViewsAmount;
			Vector3 defaultVec = Vector3(0, 0, 0);
			std::vector<Vector3> views;
			views.push_back(defaultVec);
			std::vector<Viewpoint> vs;

			bool first = true;

			std::mt19937 mt(0);
			std::normal_distribution<double> dis1(0, 1);
			std::uniform_real_distribution<double> dis_theta(0., M_PI / 2);
			std::uniform_real_distribution<double> dis_phi(-M_PI / 2, M_PI / 2);
			std::uniform_real_distribution<double> dis_distance(viewDis / 2, viewDis / 2 * 3);

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
					survey = Point3(base + viewDis * direction);
				}
				else
				{
					double new_phi = phi + dis_phi(mt);
					double new_theta = dis_theta(mt);
					view_distance_item = dis_distance(mt);

					Vector3 new_normal(
						std::sin(new_theta) * std::cos(new_phi), std::sin(new_theta) * std::sin(new_phi),
						std::cos(new_theta));
					new_normal /= std::sqrt(new_normal.squared_length());
					survey = Point3(base + view_distance_item * new_normal);
					direction = new_normal;
				}

				bool initial = true;
				int threshold = MAX_ITER_TIMES;
				double maxeva = .0;

				while (threshold >= 0)
				{
					threshold -= 1;

					// Find collision
					//bool visiable = is_visible(
					//	intrinsic_matrix,
					//	MapConverter::get_pos_pack_from_direction_vector(
					//		cgal_point_2_eigen(survey), cgal_vector_2_eigen(-direction))
					//	.camera_matrix,
					//	cgal_point_2_eigen(survey),
					//	cgal_point_2_eigen(base), tree, maxViewDis);
					bool visiable = intersectiontools::is_visible(
						intrinsic_matrix,
						MapConverter::get_pos_pack_from_direction_vector(
							cgaltools::cgal_point_2_eigen(survey), cgaltools::cgal_vector_2_eigen(-direction))
						.camera_matrix,
						cgaltools::cgal_point_2_eigen(survey),
						cgaltools::cgal_point_2_eigen(base), embree_scene, maxViewDis, mesh);


					if (direction.z() >= 0 && visiable) // Do not intersect and do not contain up view
					{
						if (threshold == MAX_ITER_TIMES - 1 && first)
						{
							bestCandidate = direction;
							disItem = viewDis;
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
					double new_theta = dis_theta(mt);
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
					vp.direction = Eigen::Vector3d(-bestCandidate.x(), -bestCandidate.y(), -bestCandidate.z()).
						normalized();
					views.push_back(bestCandidate);
					vs.push_back(vp);
				}
				viewnum--;
				if (viewnum == initialViewsAmount - 1)
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
						trajectory.push_back(v);
					}
				}
			}
			else
			{
#pragma omp critical
				{
					points.remove(i);
				}
			}
		}

		LOG(INFO) << "Finish initialization of view.";

		points.collect_garbage();
		pointsSize = static_cast<int>(points.size());
		viewSize = static_cast<int>(trajectory.size());
		LOG(INFO) << "Points num:\t" << pointsSize;
		LOG(INFO) << "Views num: \t" << viewSize;
		// *******************************************************************************************

		//auto vis = compute_visibilityIndex(trajectory, mesh, points, intrinsic_matrix, maxViewDis, &tree);
		auto vis = compute_visibilityIndex(trajectory, mesh, points, intrinsic_matrix, maxViewDis, embree_scene);

		LOG(INFO) << "Finish vis";
		std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
		comutil::checkpointTime(t);

		std::vector<Eigen::Vector3d> ptsp(pointsSize), ptsn(pointsSize);
		for (int i = 0; i < pointsSize; i++)
		{
			ptsp.at(i) = cgaltools::cgal_point_2_eigen(points.point(i));
			ptsn.at(i) = cgaltools::cgal_vector_2_eigen(points.normal(i));
		}
		LOG(INFO) << "ini";
		//auto z = totalREC(trajectory, ptsp, ptsn, vis);
		comutil::checkpointTime(t);
		//LOG(INFO) << "GPUREC: " << z;

		return 0;
	}
	else if (STAGE == 1)
		/*
		 * Do initial droneScan.
		 */
	{
		// The pointset which is processed in the workflow below.
		PointSet3 points;
		CGAL::IO::read_point_set(args["points"].asString(), points);
		for (const auto& item : points)
			points.normal(item) /= std::sqrt(points.normal(item).squared_length());

		LOG(INFO) << boost::format("Totally %d points") % points.number_of_points();

		auto finalAns = droneScan(
			points, mesh, intrinsic_matrix, MAX_ITER_TIMES, initialViewsAmount,
			logPath, viewDis, maxAngle, maxViewDis, false
		);

		/*LOG(INFO) << "Statistic of the final reconstructability";

		auto result = reconstructability_hueristic(finalAns, points, mesh,
			compute_visibility(finalAns, mesh, points, intrinsic_matrix, maxViewDis, embree_scene),
			maxViewDis, 62, 42);

		auto color_map = points.add_property_map("color",Eigen::Vector3d(0.,0.,0.)).first;
		std::vector<double> reconstructability(points.size());
		for (int i = 0; i < points.size(); i++)
		{
			reconstructability[i]=std::get<4>(result[i]);
			double color = std::min(1., std::get<4>(result[i]) / 15);
			color_map[i]=Eigen::Vector3d(color,color,color);
		}
		print_vector_distribution(reconstructability);*/

		

		std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
		std::string txttime = comutil::timeToString(t);
		write_normal_path(finalAns, logPath + txttime + "STAGE1.txt");

		comutil::debug_img();

		return 0;
	}
	else if (STAGE == 6)
	{
		PointSet3 pts(true);
		CGAL::IO::read_point_set(args["points"].asString(),pts);
		std::cout << pts.size() << std::endl;
		//load_samples_by_user(pts, args["points"].asString());
		std::vector<Viewpoint> views;
		views = read_normal_path(args["DPviews"].asString());
		//SmithAdj(views, pts, mesh, intrinsic_matrix, viewDis, fov_v);
		std::cout << views.size() << std::endl;

		SmithAdj(views, pts, mesh, intrinsic_matrix, viewDis, fov_v);
	}

	return 0;
}
