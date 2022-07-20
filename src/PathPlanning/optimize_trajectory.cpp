#include <iostream>
#include <list>
#include <boost/filesystem.hpp>
#include <argparse/argparse.hpp>
#include <json/reader.h> 
#include <glog/logging.h>
#include <filesystem>
#include <QString>
#include <QStringList>

#include "dronescan.h"
#include "DroneScanViz.h"
#include "viz.h"
#include "metrics.h"
#include "initialize.h"
#include "SmithOptimized.h"

#include "CGAL/Point_set_3.h"

Point2 lonLat2Mercator(const Point2& lonLat)
{
	Point2  mercator;
	double x = lonLat.x() * 20037508.34 / 180;
	double y = log(tan((90 + lonLat.y()) * M_PI / 360)) / (M_PI / 180);
	y = y * 20037508.34 / 180;
	/*
	//Æ«ÒÆÁ¿
	//Global_sig_2019::X_tranlate_
	//Global_sig_2019::Y_tranlate_
	//*/
	//x -= Global_sig_2019::X_tranlate_;
	//y -= Global_sig_2019::Y_tranlate_;
	////end
	mercator = Point2(x, y);
	return mercator;
}

Point2 mercator2lonLat(const Point2& mercator)
{
	Point2 lonLat;
	double x, y;
	x = mercator.x();
	//x += Global_sig_2019::X_tranlate_;
	y = mercator.y();
	//y += Global_sig_2019::Y_tranlate_;
	//end
	double x2 = x / 20037508.34 * 180;
	double y2 = y / 20037508.34 * 180;
	y2 = double(180.0 / M_PI * (2 * atan(exp(y2 * M_PI / 180.0)) - M_PI / 2.0));
	lonLat = Point2(x2, y2);
	return lonLat;
}

float normalize(Vector3& v)
{
	float len = std::sqrt(v.squared_length());
	if (len != 0.0f)
		v = v / len;
	return len;
}

bool view_direction_heading_pith_from(const FT& heading, const FT& picth, Vector3& view_direction)
{
	FT x = cos(picth * M_PI / 180) * sin(heading * M_PI / 180);
	FT y = cos(picth * M_PI / 180) * cos(heading * M_PI / 180);
	FT z = sin(picth * M_PI / 180);

	view_direction = Vector3(x, y, z);
	normalize(view_direction);
	return true;
};

bool heading_pith_from_view_direction(const Vector3& view_direction, FT& heading, FT& picth)
{
	FT length = view_direction.squared_length();

	if (length < 0.0001)
	{
		return false;
	}
	Vector3 v_tmp = view_direction;
	normalize(v_tmp);
	heading = atan2(v_tmp.x(), v_tmp.y()) * 180 / M_PI;
	picth = atan2(v_tmp.z(), sqrt(v_tmp.x() * v_tmp.x() + v_tmp.y() * v_tmp.y())) * 180 / M_PI;
	return true;
};

void load_cameras_by_user(std::vector<Viewpoint>& m_path_views_, const std::string& file_name)
{
	const int LINE_LENGTH = 500;
	char str[500];

	std::ifstream input(file_name.c_str());
	if (input.fail()) {
		std::cout << "could not open file\'" << file_name << "\'" << std::endl;
		return;
	}

	double focal35mm = 24;
	FT Horiozental_FOV, Vertical_FOV;
	Horiozental_FOV = 2 * atan(36.0 / (2 * focal35mm));
	Vertical_FOV = 2 * atan(24.0 / (2 * focal35mm));
	while (!input.eof()) {
		input.getline(str, LINE_LENGTH);
		QString sss = QString::fromStdString(str);
		sss = sss.trimmed();

		if (sss.startsWith("#"))
		{
			continue;
		}

		QString sss_tmp = sss;
		sss.replace("	", " ");
		sss.replace("  ", " ");
		while (sss.length() < sss_tmp.length())
		{
			sss_tmp = sss;
			sss = sss.replace("  ", " ");
		}

		QStringList ss_list = sss.split(" ");
		//Updated by fd 2021/8/9
		if (ss_list.size() == 5 ||
			ss_list.size() == 6)
		{
			int ID;
			double lon, lat, altitude, GPitch, DPitch, DYaw;
			std::string time_stap_date = ss_list[1].toStdString();
			std::string time_stap_time = ss_list[2].toStdString();
			lon = ss_list[0].toDouble();
			lat = ss_list[1].toDouble();
			altitude = ss_list[2].toDouble();
			GPitch = ss_list[4].toDouble();
			DYaw = ss_list[3].toDouble();
			if (ss_list.size() == 6)
			{
				bool isFited = false;
				focal35mm = ss_list[5].toDouble(&isFited);
				if (!isFited)
					focal35mm = 24;
			}

			Point2 p_lon_lat(lon, lat);
			Point2 p_Mercator = lonLat2Mercator(p_lon_lat);
			Point3 pos = Point3(p_Mercator.x(), p_Mercator.y(), altitude);
			//	Point3f pos = Point3f(lon, lat, altitude);
			Vector3 v_dir;
			view_direction_heading_pith_from(DYaw, GPitch, v_dir);

			//Updated by fd 2021/11/9
			Viewpoint tempView;
			tempView.pos_mesh = cgaltools::cgal_point_2_eigen(pos);
			tempView.direction = cgaltools::cgal_vector_2_eigen(v_dir / CGAL::sqrt(v_dir.squared_length()));
			//(pos, v_dir, Vertical_FOV, Horiozental_FOV);
			//tempView.canDelete = false;
			m_path_views_.push_back(tempView);
			//end
		}
	}
	input.close();
}
void load_samples_by_user(PointSet3& pts, const std::string& file_name)
{
	const int LINE_LENGTH = 500;
	char str[500];

	std::ifstream input(file_name.c_str());
	if (input.fail()) {
		std::cout << "could not open file\'" << file_name << "\'" << std::endl;
		return;
	}

	double focal35mm = 24;
	FT Horiozental_FOV, Vertical_FOV;
	Horiozental_FOV = 2 * atan(36.0 / (2 * focal35mm));
	Vertical_FOV = 2 * atan(24.0 / (2 * focal35mm));
	while (!input.eof()) {
		input.getline(str, LINE_LENGTH);
		QString sss = QString::fromStdString(str);
		sss = sss.trimmed();

		if (sss.startsWith("#"))
		{
			continue;
		}

		QString sss_tmp = sss;
		sss.replace("	", " ");
		sss.replace("  ", " ");
		while (sss.length() < sss_tmp.length())
		{
			sss_tmp = sss;
			sss = sss.replace("  ", " ");
		}

		QStringList ss_list = sss.split(" ");
		//Updated by fd 2021/8/9
		if (ss_list.size() == 5 ||
			ss_list.size() == 6)
		{
			int ID;
			double lon, lat, altitude, GPitch, DPitch, DYaw;
			std::string time_stap_date = ss_list[1].toStdString();
			std::string time_stap_time = ss_list[2].toStdString();
			lon = ss_list[0].toDouble();
			lat = ss_list[1].toDouble();
			altitude = ss_list[2].toDouble();
			GPitch = ss_list[4].toDouble();
			DYaw = ss_list[3].toDouble();
			if (ss_list.size() == 6)
			{
				bool isFited = false;
				focal35mm = ss_list[5].toDouble(&isFited);
				if (!isFited)
					focal35mm = 24;
			}

			Point2 p_lon_lat(lon, lat);
			Point2 p_Mercator = lonLat2Mercator(p_lon_lat);
			Point3 pos = Point3(p_Mercator.x(), p_Mercator.y(), altitude);
			//	Point3f pos = Point3f(lon, lat, altitude);
			Vector3 v_dir;
			view_direction_heading_pith_from(DYaw, GPitch, v_dir);
			
			pts.insert(pos, v_dir / CGAL::sqrt(v_dir.squared_length()));
			//end
		}
	}
	input.close();
}

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

		DroneScanViz* droneScanviz = new DroneScanViz(args["model"].asString());
		{
			droneScanviz->lock();
			droneScanviz->sample_points = points;
			droneScanviz->unlock();
		}

		auto finalAns = droneScan(
			points, mesh, intrinsic_matrix, MAX_ITER_TIMES, initialViewsAmount,
			logPath, viewDis, maxAngle, maxViewDis, droneScanviz, false
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

		{
			droneScanviz->lock();
			droneScanviz->final_reconstructability_points = points;
			droneScanviz->final_views = finalAns;
			droneScanviz->unlock();
		}

		std::chrono::system_clock::time_point t = std::chrono::system_clock::now();
		std::string txttime = comutil::timeToString(t);
		write_normal_path(finalAns, logPath + txttime + "STAGE1.txt");

		comutil::debug_img();

		return 0;
	}
	else if (STAGE == 6)
	{
		PointSet3 pts(true), vs(true);
		load_samples_by_user(pts, args["points"].asString());
		std::vector<Viewpoint> views;
		load_cameras_by_user(views, args["DPviews"].asString());
		SmithAdj(views, pts, mesh, intrinsic_matrix, viewDis, fov_v);
	}

	return 0;
}
