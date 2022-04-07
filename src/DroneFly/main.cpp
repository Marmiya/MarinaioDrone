#include <argparse/argparse.hpp>
#include <json/reader.h>
#include <opencv2/features2d.hpp>

#include "intersection_tools.h"
#include "metrics.h"
#include "trajectory.h"

#include "viz.h"
#include "next_best_target.h"
#include "mapper.h"

int main(int argc, char** argv)
{
	MapConverter map_converter;
	std::cout << "Read config " << argv[2] << std::endl;
	Json::Value args;
	std::string logPath;
	int STAGE;
	{
		google::InitGoogleLogging(argv[0]);

		argparse::ArgumentParser program("Jointly exploration, navigation and reconstruction");
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
			LOG(INFO) << err.what();
			LOG(INFO) << program;
			exit(0);
		}

		STAGE = args["stage"].asInt();
		logPath = args["logPath"].asString();
		auto logt = comutil::timeToString(std::chrono::system_clock::now());

		logPath += logt + " STAGE " + std::to_string(STAGE);
		fs::path lop(logPath);
		logPath += "/";
		create_directory(lop);

		LOG(INFO) << "Stage: " << STAGE << "\n";
		LOG(INFO) << "Log Path: " << logPath << "\n";

		fs::create_directories(logPath + "img");
		fs::create_directories(logPath + "seg");
		fs::create_directories(logPath + "point_world");
		fs::create_directories(logPath + "ccpp_map");
		fs::create_directories(logPath + "wgs_log");
		fs::create_directories(logPath + "gradually_results");
		fs::create_directories(logPath + "path");

		FLAGS_stderrthreshold = 0;
		FLAGS_log_dir = (lop).string();
	}
	args["tlogpath"] = logPath;
	bool software_parameter_is_log = args["is_log"].asBool();
	bool software_parameter_is_viz = args["is_viz"].asBool();
	FLAGS_stderrthreshold = software_parameter_is_log ? 0 : 2;

	LOG(INFO) << "Log initialization finished.";
	LOG(INFO) << "Initialization of directory, airsim, map converter and resetting color.";

	Airsim_tools* airsim_client;
	std::map<cv::Vec3b, std::string> color_to_mesh_name_map; // Color (RGB) to mesh name
	{
		auto unreal_start = Eigen::Vector3d(
			args["unreal_player_start"][0].asDouble(),
			args["unreal_player_start"][1].asDouble(),
			args["unreal_player_start"][2].asDouble()
		);
		map_converter.initDroneStart(unreal_start);

		if (args["mapper"].asString() != "gt_mapper" && args["mapper"].asString() != "graduate_gt_mapper")
		{
			airsim_client = new Airsim_tools(unreal_start);
			airsim_client->reset_color("building");
			//	airsim_client.m_agent->simSetSegmentationObjectID("BP_Sky_Sphere", 0);

			const boost::filesystem::path color_map_path(args["color_map"].asString());
			airsim_client->m_color_map = cv::imread(color_map_path.string());
			if (airsim_client->m_color_map.size == nullptr)
			{
				LOG(ERROR) << "Cannot open color map " << color_map_path << std::endl;
				return 0;
			}
			cvtColor(airsim_client->m_color_map, airsim_client->m_color_map, cv::COLOR_BGR2RGB);

			color_to_mesh_name_map = airsim_client->reset_color([](std::string v_name)
			{
				std::regex rx("^[0-9_]+$");
				bool bl = std::regex_match(v_name.begin(), v_name.end(), rx);
				return bl;
			});
		}
	}

	LOG(INFO) << "Mapping building with " << args["mapper"].asString();
	Mapper* mapper;
	if (args["mapper"] == "gt_mapper")
		mapper = new GT_mapper(args);
	else if (args["mapper"] == "graduate_gt_mapper")
		mapper = new Graduate_GT_mapper(args);
	else if (args["mapper"] == "real_mapper")
		mapper = new Real_mapper(args, airsim_client);
	else
		mapper = new Virtual_mapper(args, airsim_client, color_to_mesh_name_map);

	LOG(INFO) << "Mapper initialization finished.";

	// Some global structure

	bool end = false;
	double DRONE_STEP = args["DRONE_STEP"].asDouble();
	bool with_interpolated = args["with_interpolated"].asBool();
	const Eigen::Vector3d map_start_unreal(args["MAP_START_UNREAL_X"].asDouble(), args["MAP_START_UNREAL_Y"].asDouble(), args["MAP_START_UNREAL_Z"].asDouble());
	const Eigen::Vector3d map_end_unreal(args["MAP_END_UNREAL_X"].asDouble(), args["MAP_END_UNREAL_Y"].asDouble(), args["MAP_END_UNREAL_Z"].asDouble());
	const Eigen::Vector3d map_start_mesh(map_start_unreal.x() / 100., -map_end_unreal.y() / 100., map_start_unreal.z() / 100.);
	const Eigen::Vector3d map_end_mesh(map_end_unreal.x() / 100., -map_start_unreal.y() / 100., map_end_unreal.z() / 100.);

	Next_best_target* next_best_target;
	if (args["nbv_target"] == "Topology_decomposition")
	{
		next_best_target =
			new Next_best_target_topology_exploration(
				map_start_mesh, map_end_mesh, args["CCPP_CELL_THRESHOLD"].asInt(),
				mapper->m_boundary, args["ccpp_cell_distance"].asFloat(), args);
	}
	else if (args["nbv_target"] == "Random_min_distance")
	{
		next_best_target =
			new Next_best_target_random_min_distance(map_start_mesh, map_end_mesh, args["ccpp_cell_distance"].asFloat());
	}
	else if (args["nbv_target"] == "Exploration_first")
	{
		next_best_target =
			new Next_best_target_exploration_first(
				map_start_mesh, map_end_mesh, args["CCPP_CELL_THRESHOLD"].asInt(),
				mapper->m_boundary, args["ccpp_cell_distance"].asFloat(), args);
	}
	else
		throw;

	LOG(INFO) << "Initializing height map.";

	modeltools::Height_map runtime_height_map(map_start_mesh, map_end_mesh,
	                                          args["heightmap_resolution"].asFloat(),
	                                          args["heightmap_dilate"].asInt()
	);
	modeltools::Height_map safezone_height_map = runtime_height_map;

	bool has_safe_zone = args.isMember("safe_zone_model_path");
	if (has_safe_zone)
	{
		LOG(INFO) << "Read safe zone: " << args["safe_zone_model_path"].asString();
		CGAL::Point_set_3<Point3, Vector3> safe_zone_point_cloud;
		CGAL::IO::read_point_set(args["safe_zone_model_path"].asString(), safe_zone_point_cloud);
		modeltools::Height_map original_height_map(safe_zone_point_cloud, args["heightmap_resolution"].asFloat(),
		                                           args["heightmap_dilate"].asInt());
		safezone_height_map = original_height_map;
		safezone_height_map.save_height_map_mesh(logPath);
	}

	bool with_exploration = args["with_exploration"].asBool();
	bool with_reconstruction = args["with_reconstruction"].asBool();
	bool is_interpolated = false;
	if (!with_exploration)
	{
		runtime_height_map = safezone_height_map;
	}
	if (!with_exploration && !with_reconstruction)
		throw;

	double vertical_step = 0., horizontal_step = 0., split_min_distance = 0.; // Calculated trajectory intrinsic
	calculate_trajectory_intrinsic(args, horizontal_step, vertical_step, split_min_distance);

	std::vector<Building> total_buildings; // Map result
	std::vector<MyViewpoint> total_passed_trajectory; // Trajectory result
	double reconstruction_length = 0.;
	double exploration_length = 0.;
	double max_turn = 0.;

	Pos_Pack current_pos =
		map_converter.get_pos_pack_from_unreal(map_start_mesh, -M_PI / 2., 63. / 180. * M_PI);

	int cur_frame_id = 0;
	int building_num_record = -1;
	int current_building_num = 0;
	MyViewpoint next_viewpoint;
	Tree tree;

	// Prototype DroneFly.
	if(STAGE == 0)
	{
		//auto viz = new oriVisualizer;
		auto viz = new tVisualizer(args["obj_path"].asString());
		viz->m_uncertainty_map_distance = args["ccpp_cell_distance"].asFloat();

		while (!end)
		{
			LOG(INFO) << ">>>>>>>>>>> Frame " << cur_frame_id << " begin.  <<<<<<<<<<<";
			auto t = comutil::recordTime();

			//if(next_best_target->m_motion_status==Motion_status::exploration)
			mapper->get_buildings(total_buildings, current_pos, cur_frame_id, runtime_height_map);

			std::vector<RotatedBox> cur_boxes;
			for (const auto& item : total_buildings)
				cur_boxes.push_back(item.bounding_box_3d);

			SurfaceMesh cur_mesh = modeltools::get_rotated_box_mesh(cur_boxes);
			CGAL::IO::write_PLY(logPath + "curMesh.ply", cur_mesh);
			tree = Tree(cur_mesh.faces().begin(), cur_mesh.faces().end(), cur_mesh);

			next_best_target->update_uncertainty(current_pos, total_buildings);

			comutil::checkpointTime(t, "Height map", software_parameter_is_log);

			std::vector<MyViewpoint> current_trajectory;

			if (!with_interpolated || (with_interpolated && !is_interpolated))
			{
				// Generating trajectory
				// Input: Building vectors (std::vector<Building>)
				// Output: Modified Building.trajectory and return the whole trajectory

				current_trajectory = generate_trajectory(
					args, total_buildings,
					runtime_height_map,
					//args["mapper"].asString() == "gt_mapper" ? runtime_height_map : runtime_height_map,
					vertical_step, horizontal_step, split_min_distance, tree
				);

				LOG(INFO) << "New trajectory!";

				// Determine next position
				{
					next_viewpoint = next_best_target->determine_next_target(
						cur_frame_id, current_pos, total_buildings,
						with_exploration, horizontal_step / 2
					);

					LOG(INFO) << "Determine next position.";
				}

				// End
				if (next_best_target->m_motion_status == done) {
					break;
				}

				LOG(INFO) <<
					(boost::format("Current mode: %s. Building progress: %d/%d")
						% std::to_string(next_best_target->m_motion_status)
						% current_building_num % total_buildings.size()).str();
			}

			comutil::checkpointTime(t, "Generate trajectory", software_parameter_is_log);

			// Statics
			{
				if (cur_frame_id > 1)
				{
					double distance = (next_viewpoint.pos_mesh - current_pos.pos_mesh).norm();
					if (next_best_target->m_motion_status == exploration || next_best_target->m_motion_status == final_check) {
						exploration_length += distance;
					}
					else {
						reconstruction_length += distance;
					}
					max_turn = distance > max_turn ? distance : max_turn;
				}
				if (next_best_target->m_current_building_id != building_num_record)
				{
					current_building_num += 1;
					building_num_record = next_best_target->m_current_building_id;
				}
			}

			// Visualize
			if (software_parameter_is_viz)
			{
				viz->lock();
				viz->m_buildings = total_buildings;
				//if(next_best_target->m_motion_status==Motion_status::reconstruction)
				viz->m_current_building = next_best_target->m_current_building_id;
				viz->m_uncertainty_map.clear();
				for (const auto& item : next_best_target->sample_points)
				{
					int index = &item - &next_best_target->sample_points[0];
					viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
						next_best_target->region_status[index]);
				}
				viz->m_pos = current_pos.pos_mesh;
				viz->m_direction = current_pos.direction;
				//viz->m_trajectories = current_trajectory;
				viz->m_trajectories = total_passed_trajectory;
				//viz->m_is_reconstruction_status = trajectory_flag;
				//viz->m_trajectories_spline = total_passed_trajectory;
				//viz.m_polygon = next_best_target->img_polygon;
				viz->unlock();
				//override_sleep(0.1);
			}

			comutil::checkpointTime(t, "Viz", software_parameter_is_log);

			{
				Eigen::Vector3d direction = next_viewpoint.pos_mesh - current_pos.pos_mesh;
				Eigen::Vector3d next_direction, next_direction_temp;
				Eigen::Vector3d next_pos;

				int interpolated_num = static_cast<int>(direction.norm() / DRONE_STEP);

				if (direction.norm() < 2 * DRONE_STEP || !with_interpolated)
				{
					//next_direction = next_pos_direction.second.normalized();
					//next_direction_temp = next_viewpoint.direction;
					next_direction = next_viewpoint.direction;
					next_pos = next_viewpoint.pos_mesh;
					is_interpolated = false;
				}
				else // Bug here
				{
					LOG(INFO) << "BUG here!";

					next_direction = direction.normalized();
					next_pos = current_pos.pos_mesh + next_direction * DRONE_STEP;
					Eigen::Vector2d next_2D_direction(next_viewpoint.direction.x(), next_viewpoint.direction.y());
					Eigen::Vector2d current_2D_direction(current_pos.direction.x(), current_pos.direction.y());

					double current_yaw = atan2(current_2D_direction.y(), current_2D_direction.x());
					double next_direction_yaw = atan2(next_2D_direction.y(), next_2D_direction.x());
					double angle_delta;
					if (abs(current_yaw - next_direction_yaw) > M_PI)
					{
						if (current_yaw > next_direction_yaw)
						{
							angle_delta = (next_direction_yaw - current_yaw + M_PI * 2) / interpolated_num;
							next_direction.x() = cos(current_yaw + angle_delta);
							next_direction.y() = sin(current_yaw + angle_delta);
						}
						else
						{
							angle_delta = (current_yaw - next_direction_yaw + M_PI * 2) / interpolated_num;
							next_direction.x() = cos(current_yaw - angle_delta + M_PI * 2);
							next_direction.y() = sin(current_yaw - angle_delta + M_PI * 2);
						}
					}
					else
					{
						if (current_yaw > next_direction_yaw)
						{
							angle_delta = (current_yaw - next_direction_yaw) / interpolated_num;
							next_direction.x() = cos(current_yaw - angle_delta);
							next_direction.y() = sin(current_yaw - angle_delta);
						}
						else
						{
							angle_delta = (next_direction_yaw - current_yaw) / interpolated_num;
							next_direction.x() = cos(current_yaw + angle_delta);
							next_direction.y() = sin(current_yaw + angle_delta);
						}
					}
					next_direction.z() = -std::sqrt(next_direction.x() * next_direction.x() + next_direction.y() * next_direction.y()) * std::tan(45. / 180 * M_PI);
					next_direction.normalize();
					is_interpolated = true;
				}

				total_passed_trajectory.push_back(next_viewpoint);

				//std::ofstream pose("D:/test_data/" + std::to_string(cur_frame_id) + ".txt");
				//pose << next_pos << next_direction_temp;
				//pose.close();

				double pitch = -std::atan2f(
					next_direction[2],
					std::sqrtf(
						next_direction[0] * next_direction[0] + next_direction[1] * next_direction[1]
					)
				);
				double yaw = std::atan2f(next_direction[1], next_direction[0]);

				current_pos = map_converter.get_pos_pack_from_mesh(next_pos, yaw, pitch);

				cur_frame_id++;
			}

			comutil::checkpointTime(t, "Find next move", software_parameter_is_log);

			LOG(INFO) << "<<<<<<<<<<< Frame " << cur_frame_id - 1 << " finish. >>>>>>>>>>>\n";

			std::vector<RotatedBox> boxes;

			if (cur_frame_id % 50 == 0)
			{
				for (const auto& item : total_buildings)
					boxes.push_back(item.bounding_box_3d);
				//SurfaceMesh mesh = get_box_mesh(boxes);
				SurfaceMesh mesh = modeltools::get_rotated_box_mesh(boxes);

				CGAL::IO::write_PLY(logPath + "gradually_results/box" + std::to_string(cur_frame_id) + ".ply", mesh);
				write_normal_path_with_flag(total_passed_trajectory, logPath + "gradually_results/camera_normal_" + std::to_string(cur_frame_id) + ".log");
			}
		}

		// Done
		{
			viz->lock();
			viz->m_buildings = total_buildings;
			viz->m_pos = total_passed_trajectory[total_passed_trajectory.size() - 1].pos_mesh;
			viz->m_direction = Eigen::Vector3d(0, 0, 1);
			viz->m_trajectories.clear();
			viz->m_trajectories = total_passed_trajectory;

			if (viz->m_is_reconstruction_status.size() == 0)
				viz->m_is_reconstruction_status.resize(viz->m_trajectories.size(), 1);
			viz->m_uncertainty_map.clear();
			for (const auto& item : next_best_target->sample_points)
			{
				int index = &item - &next_best_target->sample_points[0];
				viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
					next_best_target->region_status[index]);
			}
			viz->unlock();
		}
		//total_passed_trajectory.pop_back();

		std::vector<RotatedBox> boxes;
		for (const auto& item : total_buildings)
			boxes.push_back(item.bounding_box_3d);
		//SurfaceMesh mesh = get_box_mesh(boxes);
		SurfaceMesh mesh = modeltools::get_rotated_box_mesh(boxes);
		CGAL::IO::write_PLY(logPath + "proxy.ply", mesh);

		boxes.clear();
		// Uncertainty
		std::vector<Eigen::AlignedBox3d> boxess1;
		std::vector<cv::Vec3b> boxess1_color;
		std::vector<Eigen::AlignedBox3d> boxess2;
		std::vector<cv::Vec3b> boxess2_color;
		for (const auto& item : next_best_target->sample_points)
		{
			int index = &item - &next_best_target->sample_points[0];
			if (next_best_target->region_status[index] == cv::Vec3b(0, 255, 0))
			{
				boxess1.push_back(Eigen::AlignedBox3d(
					Eigen::Vector3d(item.x() - args["ccpp_cell_distance"].asFloat() / 2,
						item.y() - args["ccpp_cell_distance"].asFloat() / 2, -1),
					Eigen::Vector3d(item.x() + args["ccpp_cell_distance"].asFloat() / 2,
						item.y() + args["ccpp_cell_distance"].asFloat() / 2, 1)));
				boxess1_color.push_back(next_best_target->region_status[index]);
			}
			else
			{
				boxess2_color.push_back(next_best_target->region_status[index]);
				boxess2.push_back(Eigen::AlignedBox3d(
					Eigen::Vector3d(item.x() - args["ccpp_cell_distance"].asFloat() / 2,
						item.y() - args["ccpp_cell_distance"].asFloat() / 2, -1),
					Eigen::Vector3d(item.x() + args["ccpp_cell_distance"].asFloat() / 2,
						item.y() + args["ccpp_cell_distance"].asFloat() / 2, 1)));
			}
		}
		modeltools::get_box_mesh_with_colors(boxess1, boxess1_color, logPath + "uncertainty_map1.obj");
		modeltools::get_box_mesh_with_colors(boxess2, boxess2_color, logPath + "uncertainty_map2.obj");

		LOG(ERROR) << "Total path num: " << total_passed_trajectory.size();
		LOG(ERROR) << "Total path length: " << evaluate_length(total_passed_trajectory);
		LOG(ERROR) << "Total exploration length: " << exploration_length;
		LOG(ERROR) << "Total reconstruction length: " << reconstruction_length;
		LOG(ERROR) << "Max_turn: " << max_turn;
		LOG(ERROR) << "Vertical step: " << vertical_step;
		LOG(ERROR) << "Horizontal step: " << horizontal_step;
		LOG(ERROR) << "Split minimum distance: " << split_min_distance;
		runtime_height_map.save_height_map_tiff(logPath + "height_map.tiff");

		comutil::debug_img();

		if (args["output_waypoint"].asBool())
		{
			total_passed_trajectory = ensure_global_safe(total_passed_trajectory, runtime_height_map,
				args["safe_distance"].asFloat(), mapper->m_boundary);
		}

		write_unreal_path(total_passed_trajectory, logPath + "path" + "/camera_after_transaction.log");
		write_smith_path(total_passed_trajectory, logPath + "path" + "/camera_smith_invert_x.log");
		write_normal_path_with_flag(total_passed_trajectory, logPath + "path" + "/camera_with_flag.log");
		LOG(ERROR) << "Write trajectory done!";

		std::vector<MyViewpoint> safe_global_trajectory;
		safe_global_trajectory = ensure_three_meter_dji(simplify_path_reduce_waypoints(total_passed_trajectory),
			safezone_height_map, args["safe_distance"].asFloat());
		write_wgs_path(args, safe_global_trajectory, logPath + "path/");
		LOG(ERROR) << "Total waypoint length: " << evaluate_length(safe_global_trajectory);
		LOG(ERROR) << "Total waypoint num: " << safe_global_trajectory.size();

		{
			viz->lock();
			viz->m_buildings = total_buildings;
			viz->m_pos = total_passed_trajectory[0].pos_mesh;
			viz->m_direction = total_passed_trajectory[0].direction;
			viz->m_trajectories.clear();
			viz->m_trajectories = safe_global_trajectory;
			if (viz->m_is_reconstruction_status.size() == 0)
				viz->m_is_reconstruction_status.resize(viz->m_trajectories.size(), 1);
			viz->m_uncertainty_map.clear();
			for (const auto& item : next_best_target->sample_points)
			{
				int index = &item - &next_best_target->sample_points[0];
				viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
					next_best_target->region_status[index]);
			}
			viz->unlock();
			//override_sleep(100);
			//debug_img(std::vector<cv::Mat>{height_map.m_map_dilated});
		}
		comutil::debug_img();
	}
	// IBS testing.
	else if (STAGE == 1)
	{
		mapper->get_buildings(total_buildings, current_pos, cur_frame_id, runtime_height_map);
		LOG(INFO) << "Buildings' num: " << total_buildings.size();
		const double safeHeight = args["safe_height"].asDouble();
		const double safeDis = args["safe_distance"].asDouble();
		const double viewDis = args["view_distance"].asDouble();
		const double unitArea = args["unit_area"].asDouble();

		std::vector<SurfaceMesh> SMs;
		SurfaceMesh cur_mesh;
		
		for (const auto& item : total_buildings)
		{
			SurfaceMesh cursm = item.buildingMesh;
			SMs.push_back(cursm);
			cur_mesh += cursm;
		}
		LOG(INFO) << "Begin IBS creating...";
		auto t = comutil::recordTime();
		SurfaceMesh ans = IBSCreatingWithSenceBB(SMs, 15., unitArea, safeDis);
		comutil::checkpointTime(t, "IBS was created");

		//auto triAns = IBSTriangulation(ans);
		//CGAL::IO::write_PLY(logPath + "triIBS.ply", triAns);
		CGAL::IO::write_PLY(logPath + "curIBS.ply", ans);
		CGAL::IO::write_PLY(logPath + "curMesh.ply", cur_mesh);

		comutil::checkpointTime(t, "Begin views sampling...");
		
		PointSet3 views = IBSviewNet(ans, SMs, safeDis, safeHeight, 5., 5.);

		std::vector<Viewpoint> vs;
		for (const auto& i : views)
		{
			Viewpoint tv;
			tv.pos_mesh = cgaltools::cgal_point_2_eigen(views.point(i));
			tv.direction = cgaltools::cgal_vector_2_eigen(views.normal(i));
			vs.push_back(tv);
		}
		write_smith18_path(vs, logPath + "smithPath.log");
		comutil::checkpointTime(t, "Views sampling was finished");
		CGAL::IO::write_point_set(logPath + "IBSviews.ply", views);
	}
	// IBS-based Planar.
	else if (STAGE == 2)
	{
		auto viz = new tVisualizer(args["obj_path"].asString());
		viz->m_uncertainty_map_distance = args["ccpp_cell_distance"].asDouble();

		while (!end)
		{
			LOG(INFO) << ">>>>>>>>>>> Frame " << cur_frame_id << " begin.  <<<<<<<<<<<";
			auto t = comutil::recordTime();

			mapper->get_buildings(total_buildings, current_pos, cur_frame_id, runtime_height_map);
			LOG(INFO) << "Buildings' num: " << total_buildings.size();

			std::vector<SurfaceMesh> SMs;
			SurfaceMesh cur_mesh;
			for (const auto& item : total_buildings)
			{
				SMs.push_back(item.buildingMesh);
				cur_mesh += item.buildingMesh;
			}
			CGAL::IO::write_PLY(logPath + "curMesh.ply", cur_mesh);

			tree = Tree(cur_mesh.faces().begin(), cur_mesh.faces().end(), cur_mesh);
			next_best_target->update_uncertainty(current_pos, total_buildings);

			std::vector<MyViewpoint> current_trajectory;

			if (!with_interpolated || (with_interpolated && !is_interpolated))
			{
				current_trajectory = generate_trajectory_tg(
					args, total_buildings,
					runtime_height_map, tree
				);

				LOG(INFO) << "New trajectory!";

				// Determine next position
				{
					next_viewpoint = next_best_target->determine_next_target(
						cur_frame_id, current_pos, total_buildings,
						with_exploration, horizontal_step / 2
					);

					LOG(INFO) << "Determine next position.";
				}

				// End
				if (next_best_target->m_motion_status == done) {
					break;
				}

				LOG(INFO) <<
					(boost::format("Current mode: %s. Building progress: %d/%d")
						% std::to_string(next_best_target->m_motion_status)
						% current_building_num % total_buildings.size()).str();
			}

			comutil::checkpointTime(t, "Generate trajectory", software_parameter_is_log);

			// Statics
			{
				if (cur_frame_id > 1)
				{
					double distance = (next_viewpoint.pos_mesh - current_pos.pos_mesh).norm();
					if (next_best_target->m_motion_status == exploration || next_best_target->m_motion_status == final_check) {
						exploration_length += distance;
					}
					else {
						reconstruction_length += distance;
					}
					max_turn = distance > max_turn ? distance : max_turn;
				}
				if (next_best_target->m_current_building_id != building_num_record)
				{
					current_building_num += 1;
					building_num_record = next_best_target->m_current_building_id;
				}
			}

			// Visualize
			if (software_parameter_is_viz)
			{
				viz->lock();
				viz->m_buildings = total_buildings;
				//if(next_best_target->m_motion_status==Motion_status::reconstruction)
				viz->m_current_building = next_best_target->m_current_building_id;
				viz->m_uncertainty_map.clear();
				for (const auto& item : next_best_target->sample_points)
				{
					int index = &item - &next_best_target->sample_points[0];
					viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
						next_best_target->region_status[index]);
				}
				viz->m_pos = current_pos.pos_mesh;
				viz->m_direction = current_pos.direction;
				//viz->m_trajectories = current_trajectory;
				viz->m_trajectories = total_passed_trajectory;
				//viz->m_is_reconstruction_status = trajectory_flag;
				//viz->m_trajectories_spline = total_passed_trajectory;
				//viz.m_polygon = next_best_target->img_polygon;
				viz->unlock();
				//override_sleep(0.1);
			}

			comutil::checkpointTime(t, "Viz", software_parameter_is_log);

			{
				Eigen::Vector3d direction = next_viewpoint.pos_mesh - current_pos.pos_mesh;
				Eigen::Vector3d next_direction, next_direction_temp;
				Eigen::Vector3d next_pos;

				int interpolated_num = static_cast<int>(direction.norm() / DRONE_STEP);

				if (direction.norm() < 2 * DRONE_STEP || !with_interpolated)
				{
					//next_direction = next_pos_direction.second.normalized();
					//next_direction_temp = next_viewpoint.direction;
					next_direction = next_viewpoint.direction;
					next_pos = next_viewpoint.pos_mesh;
					is_interpolated = false;
				}
				else // Bug here
				{
					LOG(INFO) << "BUG here!";

					next_direction = direction.normalized();
					next_pos = current_pos.pos_mesh + next_direction * DRONE_STEP;
					Eigen::Vector2d next_2D_direction(next_viewpoint.direction.x(), next_viewpoint.direction.y());
					Eigen::Vector2d current_2D_direction(current_pos.direction.x(), current_pos.direction.y());

					double current_yaw = atan2(current_2D_direction.y(), current_2D_direction.x());
					double next_direction_yaw = atan2(next_2D_direction.y(), next_2D_direction.x());
					double angle_delta;
					if (abs(current_yaw - next_direction_yaw) > M_PI)
					{
						if (current_yaw > next_direction_yaw)
						{
							angle_delta = (next_direction_yaw - current_yaw + M_PI * 2) / interpolated_num;
							next_direction.x() = cos(current_yaw + angle_delta);
							next_direction.y() = sin(current_yaw + angle_delta);
						}
						else
						{
							angle_delta = (current_yaw - next_direction_yaw + M_PI * 2) / interpolated_num;
							next_direction.x() = cos(current_yaw - angle_delta + M_PI * 2);
							next_direction.y() = sin(current_yaw - angle_delta + M_PI * 2);
						}
					}
					else
					{
						if (current_yaw > next_direction_yaw)
						{
							angle_delta = (current_yaw - next_direction_yaw) / interpolated_num;
							next_direction.x() = cos(current_yaw - angle_delta);
							next_direction.y() = sin(current_yaw - angle_delta);
						}
						else
						{
							angle_delta = (next_direction_yaw - current_yaw) / interpolated_num;
							next_direction.x() = cos(current_yaw + angle_delta);
							next_direction.y() = sin(current_yaw + angle_delta);
						}
					}
					next_direction.z() = -std::sqrt(next_direction.x() * next_direction.x() + next_direction.y() * next_direction.y()) * std::tan(45. / 180 * M_PI);
					next_direction.normalize();
					is_interpolated = true;
				}

				total_passed_trajectory.push_back(next_viewpoint);

				double pitch = -std::atan2f(
					next_direction[2],
					std::sqrtf(
						next_direction[0] * next_direction[0] + next_direction[1] * next_direction[1]
					)
				);
				double yaw = std::atan2f(next_direction[1], next_direction[0]);

				current_pos = map_converter.get_pos_pack_from_mesh(next_pos, yaw, pitch);

				cur_frame_id++;
			}

			comutil::checkpointTime(t, "Find next move", software_parameter_is_log);

			LOG(INFO) << "<<<<<<<<<<< Frame " << cur_frame_id - 1 << " finish. >>>>>>>>>>>\n";

			std::vector<RotatedBox> boxes;

			if (cur_frame_id % 50 == 0)
			{
				for (const auto& item : total_buildings)
					boxes.push_back(item.bounding_box_3d);
				//SurfaceMesh mesh = get_box_mesh(boxes);
				SurfaceMesh mesh = modeltools::get_rotated_box_mesh(boxes);

				CGAL::IO::write_PLY(logPath + "gradually_results/box" + std::to_string(cur_frame_id) + ".ply", mesh);
				write_normal_path_with_flag(total_passed_trajectory, logPath + "gradually_results/camera_normal_" + std::to_string(cur_frame_id) + ".log");
			}
		}

		// Done
		{
			viz->lock();
			viz->m_buildings = total_buildings;
			viz->m_pos = total_passed_trajectory[total_passed_trajectory.size() - 1].pos_mesh;
			viz->m_direction = Eigen::Vector3d(0, 0, 1);
			viz->m_trajectories.clear();
			viz->m_trajectories = total_passed_trajectory;

			if (viz->m_is_reconstruction_status.size() == 0)
				viz->m_is_reconstruction_status.resize(viz->m_trajectories.size(), 1);
			viz->m_uncertainty_map.clear();
			for (const auto& item : next_best_target->sample_points)
			{
				int index = &item - &next_best_target->sample_points[0];
				viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
					next_best_target->region_status[index]);
			}
			viz->unlock();
		}
		//total_passed_trajectory.pop_back();

		std::vector<RotatedBox> boxes;
		for (const auto& item : total_buildings)
			boxes.push_back(item.bounding_box_3d);
		//SurfaceMesh mesh = get_box_mesh(boxes);
		SurfaceMesh mesh = modeltools::get_rotated_box_mesh(boxes);
		CGAL::IO::write_PLY(logPath + "proxy.ply", mesh);

		boxes.clear();
		// Uncertainty
		std::vector<Eigen::AlignedBox3d> boxess1;
		std::vector<cv::Vec3b> boxess1_color;
		std::vector<Eigen::AlignedBox3d> boxess2;
		std::vector<cv::Vec3b> boxess2_color;
		for (const auto& item : next_best_target->sample_points)
		{
			int index = &item - &next_best_target->sample_points[0];
			if (next_best_target->region_status[index] == cv::Vec3b(0, 255, 0))
			{
				boxess1.push_back(Eigen::AlignedBox3d(
					Eigen::Vector3d(item.x() - args["ccpp_cell_distance"].asFloat() / 2,
						item.y() - args["ccpp_cell_distance"].asFloat() / 2, -1),
					Eigen::Vector3d(item.x() + args["ccpp_cell_distance"].asFloat() / 2,
						item.y() + args["ccpp_cell_distance"].asFloat() / 2, 1)));
				boxess1_color.push_back(next_best_target->region_status[index]);
			}
			else
			{
				boxess2_color.push_back(next_best_target->region_status[index]);
				boxess2.push_back(Eigen::AlignedBox3d(
					Eigen::Vector3d(item.x() - args["ccpp_cell_distance"].asFloat() / 2,
						item.y() - args["ccpp_cell_distance"].asFloat() / 2, -1),
					Eigen::Vector3d(item.x() + args["ccpp_cell_distance"].asFloat() / 2,
						item.y() + args["ccpp_cell_distance"].asFloat() / 2, 1)));
			}
		}
		modeltools::get_box_mesh_with_colors(boxess1, boxess1_color, logPath + "uncertainty_map1.obj");
		modeltools::get_box_mesh_with_colors(boxess2, boxess2_color, logPath + "uncertainty_map2.obj");

		LOG(ERROR) << "Total path num: " << total_passed_trajectory.size();
		LOG(ERROR) << "Total path length: " << evaluate_length(total_passed_trajectory);
		LOG(ERROR) << "Total exploration length: " << exploration_length;
		LOG(ERROR) << "Total reconstruction length: " << reconstruction_length;
		LOG(ERROR) << "Max_turn: " << max_turn;
		LOG(ERROR) << "Vertical step: " << vertical_step;
		LOG(ERROR) << "Horizontal step: " << horizontal_step;
		LOG(ERROR) << "Split minimum distance: " << split_min_distance;
		runtime_height_map.save_height_map_tiff(logPath + "height_map.tiff");

		comutil::debug_img();

		if (args["output_waypoint"].asBool())
		{
			total_passed_trajectory = ensure_global_safe(total_passed_trajectory, runtime_height_map,
				args["safe_distance"].asFloat(), mapper->m_boundary);
		}

		write_unreal_path(total_passed_trajectory, logPath + "path" + "/camera_after_transaction.log");
		write_smith_path(total_passed_trajectory, logPath + "path" + "/camera_smith_invert_x.log");
		write_normal_path_with_flag(total_passed_trajectory, logPath + "path" + "/camera_with_flag.log");
		LOG(ERROR) << "Write trajectory done!";

		std::vector<MyViewpoint> safe_global_trajectory;
		safe_global_trajectory = ensure_three_meter_dji(simplify_path_reduce_waypoints(total_passed_trajectory),
			safezone_height_map, args["safe_distance"].asFloat());
		write_wgs_path(args, safe_global_trajectory, logPath + "path/");
		LOG(ERROR) << "Total waypoint length: " << evaluate_length(safe_global_trajectory);
		LOG(ERROR) << "Total waypoint num: " << safe_global_trajectory.size();

		{
			viz->lock();
			viz->m_buildings = total_buildings;
			viz->m_pos = total_passed_trajectory[0].pos_mesh;
			viz->m_direction = total_passed_trajectory[0].direction;
			viz->m_trajectories.clear();
			viz->m_trajectories = safe_global_trajectory;
			if (viz->m_is_reconstruction_status.size() == 0)
				viz->m_is_reconstruction_status.resize(viz->m_trajectories.size(), 1);
			viz->m_uncertainty_map.clear();
			for (const auto& item : next_best_target->sample_points)
			{
				int index = &item - &next_best_target->sample_points[0];
				viz->m_uncertainty_map.emplace_back(Eigen::Vector2d(item.x(), item.y()),
					next_best_target->region_status[index]);
			}
			viz->unlock();
			
		}
		comutil::debug_img();
	}
	// Stage which is not defined.
	else
	{
		throw;
	}

	return 0;
}
