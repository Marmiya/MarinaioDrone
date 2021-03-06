#include "mapper.h"

const cv::Vec3b BACKGROUND_COLOR(57, 181, 55);
const cv::Vec3b SKY_COLOR(161, 120, 205);
const int MAX_FEATURES = 100000;

/*
 * Mapper
 */

Mapper::Mapper(const Json::Value& v_args) : m_args(v_args)
{
	std::vector<Point2> points;

	if (!v_args["boundary"].isNull())
	{
		const int num_point = v_args["boundary"].size() / 3;
		for (int i_point = 0; i_point < num_point; ++i_point)
		{
			Point2 p_lonlat(v_args["boundary"][i_point * 3 + 0].asDouble(),
				v_args["boundary"][i_point * 3 + 1].asDouble());
			Eigen::Vector2d p_mercator =
				lonLat2Mercator(Eigen::Vector2d(p_lonlat.x(), p_lonlat.y())) -
				Eigen::Vector2d(v_args["geo_origin"][0].asDouble(), v_args["geo_origin"][1].asDouble());
			points.emplace_back(p_mercator.x(), p_mercator.y());
		}
		m_boundary = Polygon2(points.begin(), points.end());
	}
	else
	{
		std::vector<Point2> points;
		const Eigen::Vector3d map_start_mesh(m_args["MAP_START_UNREAL_X"].asDouble() / 100.,
			-m_args["MAP_START_UNREAL_Y"].asDouble() / 100.,
			m_args["MAP_START_UNREAL_Z"].asDouble() / 100.);
		const Eigen::Vector3d map_end_mesh(m_args["MAP_END_UNREAL_X"].asDouble() / 100.,
			-m_args["MAP_END_UNREAL_Y"].asDouble() / 100.,
			m_args["MAP_END_UNREAL_Z"].asDouble() / 100.);

		points.emplace_back(map_start_mesh.x(), map_start_mesh.y());
		points.emplace_back(map_start_mesh.x(), map_end_mesh.y());
		points.emplace_back(map_end_mesh.x(), map_end_mesh.y());
		points.emplace_back(map_end_mesh.x(), map_start_mesh.y());

		m_boundary = Polygon2(points.begin(), points.end());
	}
}

/*
 * GTMapper
 */

GT_mapper::GT_mapper(const Json::Value& args) : Mapper(args)
{
	PointSet3 original_point_cloud(true);
	std::vector<CGAL::Point_set_3<Point3, Vector3>> pcs;
	if (boost::filesystem::is_directory(args["model_path"].asString()))
	{
		boost::filesystem::directory_iterator end_iter;
		for (fs::directory_iterator iter(args["model_path"].asString()); iter != end_iter; ++iter)
		{
			CGAL::Point_set_3<Point3, Vector3> pc_item;
			CGAL::IO::read_point_set(iter->path().string(), pc_item);
			pcs.push_back(pc_item);
		}
		
		m_buildings_safe_place.resize(pcs.size());
		for (int cluster_id = 0; cluster_id < pcs.size(); ++cluster_id)
		{
			Building& current_building = m_buildings_safe_place[cluster_id];
			bool add_as_target = true;
			for (PointSet3::Index idx : pcs[cluster_id])
			{
				current_building.points_world_space.insert(pcs[cluster_id].point(idx));
				Point2 p(pcs[cluster_id].point(idx).x(), pcs[cluster_id].point(idx).y());
				if (m_boundary.size() > 0)
				{
					for (auto iter_segment = m_boundary.edges_begin(); iter_segment != m_boundary.edges_end();
						++iter_segment)
						if (CGAL::squared_distance(p, *iter_segment) < 0 * 0)
							add_as_target = false;
					if (m_boundary.bounded_side(p) != CGAL::ON_BOUNDED_SIDE)
						add_as_target = false;
				}
			}
			
			current_building.bounding_box_3d = cgaltools::get_bounding_box_rotated(
				current_building.points_world_space);
			current_building.boxes.push_back(current_building.bounding_box_3d);

			if (add_as_target)
				m_buildings_target.push_back(current_building);
		}
		return;
	}

	CGAL::IO::read_point_set(args["model_path"].asString(), original_point_cloud);
	LOG(INFO) << "Model reading finished.";

	PointSet3 point_cloud(original_point_cloud);
	if (modeltools::liftModel(point_cloud))
	{
		CGAL::IO::write_point_set(args["tlogpath"].asString() + "liftedPTS.ply", point_cloud);
		if (args["ori_lift"].asBool()) {
			SurfaceMesh msh = modeltools::read_model(args["obj_path"].asString());
			double lowestPoint = 9999999.;
			for (const auto& i : msh.points())
			{
				lowestPoint = i.z() < lowestPoint ? i.z() : lowestPoint;
			}
			LOG(INFO) << "LOWEST point: " << lowestPoint;
			for (auto& i : msh.points())
			{
				i = Point3(i.x(), i.y(), i.z() + ::abs(lowestPoint));
			}
			CGAL::IO::write_PLY(args["obj_path"].asString(), msh);
		}

	}
	// Delete ground planes

	for (int idx = point_cloud.size() - 1; idx >= 0; idx--)
	{
		if (point_cloud.point(idx).z() <= args["HEIGHT_CLIP"].asDouble())
			point_cloud.remove(idx);
	}

	point_cloud.collect_garbage();

	CGAL::IO::write_point_set(args["tlogpath"].asString() + "points_without_plane.ply", point_cloud);
	// Cluster building
	std::size_t nb_clusters;
	{
		PointSet3::Property_map<int> cluster_map = point_cloud.add_property_map<int>("cluster", -1).first;

		std::vector<std::pair<std::size_t, std::size_t>> adjacencies;

		nb_clusters = cluster_point_set(point_cloud, cluster_map,
			point_cloud.parameters().neighbor_radius(
				args["cluster_radius"].asDouble()).
			adjacencies(std::back_inserter(adjacencies)));

		m_buildings_target.resize(nb_clusters);
		LOG(INFO) << "Clustering num: " << nb_clusters;

		PointSet3::Property_map<unsigned char> red = point_cloud.add_property_map<unsigned char>("red", 0).first;
		PointSet3::Property_map<unsigned char> green = point_cloud.add_property_map<unsigned char>("green", 0).first;
		PointSet3::Property_map<unsigned char> blue = point_cloud.add_property_map<unsigned char>("blue", 0).first;
		for (PointSet3::Index idx : point_cloud)
		{
			// One color per cluster
			int cluster_id = cluster_map[idx];
			CGAL::Random rand(cluster_id);
			red[idx] = rand.get_int(64, 192);
			green[idx] = rand.get_int(64, 192);
			blue[idx] = rand.get_int(64, 192);

			Building& current_building = m_buildings_target[cluster_id];
			current_building.points_world_space.add_normal_map();
			current_building.points_world_space.insert(point_cloud.point(idx), point_cloud.normal(idx));
		}
	}

	#pragma omp parallel for
	for (int i = 0; i < m_buildings_target.size(); ++i)
	{
		m_buildings_target[i].bounding_box_3d = cgaltools::get_bounding_box_rotated(
			m_buildings_target[i].points_world_space);
		m_buildings_target[i].boxes.push_back(m_buildings_target[i].bounding_box_3d);
		PointSet3 tpts(true);
		tpts = m_buildings_target[i].points_world_space;

		double spacing = CGAL::compute_average_spacing<CGAL::Sequential_tag>(tpts, 6);

		if (args["simplify"].asBool()) 
		{
			tpts.remove(
				grid_simplify_point_set(
					tpts, spacing, CGAL::parameters::point_map(tpts.point_map())
				), tpts.end()
			);

			std::cout << tpts.number_of_removed_points()
				<< " point(s) removed after simplification." << std::endl;
			tpts.collect_garbage();
		}

		CGAL::poisson_surface_reconstruction_delaunay
		(tpts.begin(), tpts.end(), tpts.point_map(), tpts.normal_map(),
			m_buildings_target[i].buildingMesh, spacing);
		m_buildings_target[i].buildingMeshs.push_back(m_buildings_target[i].buildingMesh);

		CGAL::IO::write_PLY(args["tlogpath"].asString() + std::to_string(i) + "mesh.ply", m_buildings_target[i].buildingMesh);
	}

	for (int i_building_1 = m_buildings_target.size() - 1; i_building_1 >= 0; --i_building_1)
	{
		cv::Point2f points[4];
		m_buildings_target[i_building_1].bounding_box_3d.cv_box.points(points);

		if (m_boundary.size() > 0)
		{
			bool should_delete = false;
			for (int i_point = 0; i_point < 4; ++i_point)
			{
				Point2 p(points[i_point].x, points[i_point].y);
				for (auto iter_segment = m_boundary.edges_begin(); iter_segment != m_boundary.edges_end(); ++
					iter_segment)
					
					if (CGAL::squared_distance(p, *iter_segment) < 0)
					{
						should_delete = true;
					}

				if (m_boundary.bounded_side(p) != CGAL::ON_BOUNDED_SIDE)
				{
					should_delete = true;
				}
			}
			if (should_delete)
				m_buildings_target.erase(m_buildings_target.begin() + i_building_1);
		}
	}

	m_buildings_safe_place = m_buildings_target;
}

void GT_mapper::get_buildings(
	std::vector<Building>& v_buildings,	const Pos_Pack& v_current_pos,
	const int v_cur_frame_id, modeltools::Height_map& v_height_map
)
{
	if (v_buildings.empty())
	{
		v_buildings = m_buildings_target;
		for (auto& item_building : m_buildings_safe_place)
		{
			v_height_map.update(item_building.bounding_box_3d);
		}
	}
}

/*
 * Graduate GT Mapper
 */

Graduate_GT_mapper::Graduate_GT_mapper(const Json::Value& args) : Mapper(args)
{
	CGAL::Point_set_3<Point3, Vector3> original_point_cloud;
	std::vector<CGAL::Point_set_3<Point3, Vector3>> pcs;
	if (boost::filesystem::is_directory(args["model_path"].asString()))
	{
		boost::filesystem::directory_iterator end_iter;
		for (fs::directory_iterator iter(args["model_path"].asString()); iter != end_iter; ++iter)
		{
			CGAL::Point_set_3<Point3, Vector3> pc_item;
			CGAL::IO::read_point_set(iter->path().string(), pc_item);
			pcs.push_back(pc_item);
		}
		//CGAL::read_ply_point_set(std::ifstream("D:\\datasets\\Realcity\\Shenzhen\\sample_500000.ply", std::ios::binary), original_point_cloud);
		//CGAL::Point_set_3<Point_3, Vector_3> point_cloud(original_point_cloud);

		m_buildings_safe_place.resize(pcs.size());
		for (int cluster_id = 0; cluster_id < pcs.size(); ++cluster_id)
		{
			Building& current_building = m_buildings_safe_place[cluster_id];
			bool add_as_target = true;
			for (PointSet3::Index idx : pcs[cluster_id])
			{
				current_building.points_world_space.insert(pcs[cluster_id].point(idx));
				Point2 p(pcs[cluster_id].point(idx).x(), pcs[cluster_id].point(idx).y());
				if (m_boundary.size() > 0)
				{
					for (auto iter_segment = m_boundary.edges_begin(); iter_segment != m_boundary.edges_end();
						++iter_segment)
						if (CGAL::squared_distance(p, *iter_segment) < 00 * 00)
							add_as_target = false;
					if (m_boundary.bounded_side(p) != CGAL::ON_BOUNDED_SIDE)
						add_as_target = false;
				}
			}
			//current_building.bounding_box_3d = get_bounding_box(current_building.points_world_space);
			current_building.bounding_box_3d = cgaltools::get_bounding_box_rotated(
				current_building.points_world_space);
			current_building.boxes.push_back(current_building.bounding_box_3d);

			if (add_as_target)
				m_buildings_target.push_back(current_building);
		}
		m_is_building_add.resize(m_buildings_target.size(), false);
		return;
	}
	CGAL::IO::read_point_set(args["model_path"].asString(), original_point_cloud);
	CGAL::Point_set_3<Point3, Vector3> point_cloud(original_point_cloud);
	// Delete ground planes
	{
		for (int idx = point_cloud.size() - 1; idx >= 0; idx--)
		{
			if (point_cloud.point(idx).z() < args["HEIGHT_CLIP"].asFloat())
				point_cloud.remove(idx);
		}

		point_cloud.collect_garbage();
	}
	CGAL::IO::write_point_set("points_without_plane.ply", point_cloud);
	// Cluster building
	std::size_t nb_clusters;
	{
		PointSet3::Property_map<int> cluster_map = point_cloud.add_property_map<int>("cluster", -1).first;

		std::vector<std::pair<std::size_t, std::size_t>> adjacencies;

		nb_clusters = cluster_point_set(point_cloud, cluster_map,
			point_cloud.parameters().neighbor_radius(
				args["cluster_radius"].asFloat()).
			adjacencies(std::back_inserter(adjacencies)));
		m_buildings_target.resize(nb_clusters);

		PointSet3::Property_map<unsigned char> red = point_cloud.add_property_map<unsigned char>("red", 0).
			first;
		PointSet3::Property_map<unsigned char> green = point_cloud
			.add_property_map<unsigned char>("green", 0).first;
		PointSet3::Property_map<unsigned char> blue = point_cloud.add_property_map<unsigned char>("blue", 0).
			first;
		for (PointSet3::Index idx : point_cloud)
		{
			// One color per cluster
			int cluster_id = cluster_map[idx];
			CGAL::Random rand(cluster_id);
			red[idx] = rand.get_int(64, 192);
			green[idx] = rand.get_int(64, 192);
			blue[idx] = rand.get_int(64, 192);

			Building& current_building = m_buildings_target[cluster_id];
			current_building.points_world_space.insert(point_cloud.point(idx));
		}
	}
	for (int i_building_1 = 0; i_building_1 < m_buildings_target.size(); ++i_building_1)
	{
		//m_buildings_target[i_building_1].bounding_box_3d = get_bounding_box(m_buildings_target[i_building_1].points_world_space);
		m_buildings_target[i_building_1].bounding_box_3d = cgaltools::get_bounding_box_rotated(
			m_buildings_target[i_building_1].points_world_space);
		m_buildings_target[i_building_1].bounding_box_3d.box.min().z() -= args["HEIGHT_CLIP"].asFloat();
		m_buildings_target[i_building_1].boxes.push_back(m_buildings_target[i_building_1].bounding_box_3d);
	}

	for (int i_building_1 = m_buildings_target.size() - 1; i_building_1 >= 0; --i_building_1)
	{
		cv::Point2f points[4];
		m_buildings_target[i_building_1].bounding_box_3d.cv_box.points(points);

		if (m_boundary.size() > 0)
		{
			bool should_delete = false;
			for (int i_point = 0; i_point < 4; ++i_point)
			{
				Point2 p(points[i_point].x, points[i_point].y);
				for (auto iter_segment = m_boundary.edges_begin(); iter_segment != m_boundary.edges_end(); ++
					iter_segment)
					if (CGAL::squared_distance(p, *iter_segment) < args["safe_distance"].asFloat() * args[
						"safe_distance"].asFloat() * 2)
						should_delete = true;
						if (m_boundary.bounded_side(p) != CGAL::ON_BOUNDED_SIDE)
							should_delete = true;
			}
			if (should_delete)
				m_buildings_target.erase(m_buildings_target.begin() + i_building_1);
		}
	}
	m_buildings_safe_place = m_buildings_target;
	m_is_building_add.resize(m_buildings_target.size(), false);

}

void Graduate_GT_mapper::get_buildings(
	std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
	const int v_cur_frame_id, modeltools::Height_map& v_height_map
)
{
	if (v_buildings.size() != m_buildings_target.size())
	{
		for (auto& item_building : m_buildings_target)
		{
			v_height_map.update(item_building.bounding_box_3d);
			if (!m_is_building_add[&item_building - &m_buildings_target[0]] &&
				(Eigen::Vector2d(v_current_pos.pos_mesh.x() - item_building.bounding_box_3d.box.center().x(),
					v_current_pos.pos_mesh.y() - item_building.bounding_box_3d.box.center().y())).
				norm() < m_args["ccpp_cell_distance"].asFloat() * 3)
			{
				v_buildings.push_back(item_building);
				m_is_building_add[&item_building - &m_buildings_target[0]] = true;
			}
		}
	}
}

/*
 * Virtual Mapper
 */

std::vector<Virtual_mapper::ImageCluster>
Virtual_mapper::solveCluster(
	const cv::Mat& vSeg, const std::map<cv::Vec3b, std::string> colorMap, bool& isValid
)
{
	isValid = true;
	std::map<cv::Vec3b, int> currentColor;
	std::vector<std::pair<std::vector<int>, std::vector<int>>> bbox;

	int background_num = 0;
	for (int y = 0; y < vSeg.size().height; y++)
	{
		for (int x = 0; x < vSeg.size().width; x++)
		{
			auto pixel = vSeg.at<cv::Vec3b>(y, x);
			if (pixel == cv::Vec3b(55, 181, 57))
			{
				background_num++;
				continue;
			}

			if (!currentColor.contains(pixel))
			{
				currentColor.insert(std::make_pair(pixel, static_cast<int>(bbox.size())));
				bbox.push_back(std::make_pair(std::vector<int>(), std::vector<int>()));
			}

			bbox[currentColor.at(pixel)].first.push_back(x);
			bbox[currentColor.at(pixel)].second.push_back(y);
		}
	}

	std::vector<ImageCluster> result;
	if (background_num > vSeg.size().height * vSeg.size().width * 0.8)
		isValid = false;

	int small_building_num = 0;
	for (auto colorIter = currentColor.begin(); colorIter != currentColor.end(); ++colorIter)
	{
		ImageCluster cluster;
		if (bbox[colorIter->second].first.size() < 30 * 30)
		{
			small_building_num++;
			continue;
		}
		//if (bbox[colorIter->second].first.size() < 30 * 30)
		//{
		//	isValid = false;
		//	break;
		//}
		cluster.box = CGAL::Bbox_2(
			*std::min_element(bbox[colorIter->second].first.begin(), bbox[colorIter->second].first.end()),
			*std::min_element(bbox[colorIter->second].second.begin(), bbox[colorIter->second].second.end()),
			*std::max_element(bbox[colorIter->second].first.begin(), bbox[colorIter->second].first.end()),
			*std::max_element(bbox[colorIter->second].second.begin(), bbox[colorIter->second].second.end())
		);
		cluster.color = colorIter->first;
		if (!colorMap.contains(cluster.color))
			continue;
		cluster.name = std::to_string(std::atoi(colorMap.at(cluster.color).c_str()));
		cluster.xs = bbox[currentColor.at(colorIter->first)].first;
		cluster.ys = bbox[currentColor.at(colorIter->first)].second;
		result.push_back(cluster);
	}
	if (small_building_num > 40)
		isValid = false;
	return result;
}

std::pair<cv::RotatedRect, Point2>
Virtual_mapper::get_bbox_3d(const PointSet3& v_point_cloud)
{
	std::vector<float> vertices_z;
	std::vector<cv::Point2f> vertices_xy;
	for (auto& item_point : v_point_cloud.points())
	{
		vertices_z.push_back(item_point.z());
		vertices_xy.push_back(cv::Point2f(item_point.x(), item_point.y()));
	}
	cv::RotatedRect box = minAreaRect(vertices_xy);
	float max_z = *std::max_element(vertices_z.begin(), vertices_z.end());
	float min_z = *std::min_element(vertices_z.begin(), vertices_z.end());

	//Note: Coordinates transformation
	return std::make_pair(box, Point2(min_z, max_z));
}

float Virtual_mapper::calculate_3d_iou(
	const Building& building1, const Building& building2
)
{
	// compute intersection area
	std::vector<cv::Point2f> intersections_unsorted;
	std::vector<cv::Point2f> intersections;
	rotatedRectangleIntersection(building1.bounding_box_3d.cv_box, building2.bounding_box_3d.cv_box,
		intersections_unsorted);
	if (intersections_unsorted.size() < 3)
	{
		return 0;
	}
	// need to sort the vertices CW or CCW
	convexHull(intersections_unsorted, intersections);

	// Shoelace formula
	float intersection_area = 0;
	for (unsigned int i = 0; i < intersections.size(); ++i)
	{
		const auto& pt = intersections[i];
		const unsigned int i_next = (i + 1) == intersections.size() ? 0 : (i + 1);
		const auto& pt_next = intersections[i_next];
		intersection_area += (pt.x * pt_next.y - pt_next.x * pt.y);
	}
	intersection_area = std::abs(intersection_area) / 2;

	float intersection_volume = intersection_area * std::min(
		building1.bounding_box_3d.box.max().z() - building1.bounding_box_3d.box.min().z(),
		building2.bounding_box_3d.box.max().z() - building2.bounding_box_3d.box.min().z());

	float union_volume = building1.bounding_box_3d.box.volume() + building2.bounding_box_3d.box.volume() -
		intersection_volume;

	return intersection_volume / union_volume;
}

void Virtual_mapper::read_mesh(
	std::string in_path, std::map<string, PointSet3>& v_out_point_clouds,
	std::map<string, SurfaceMesh>& v_out_meshes
)
{
	boost::filesystem::path myPath(in_path);
	boost::filesystem::recursive_directory_iterator endIter;
	for (boost::filesystem::recursive_directory_iterator iter(myPath); iter != endIter; ++iter)
	{
		std::string v_name = iter->path().stem().string();
		std::regex rx("^[0-9]+$");
		bool bl = std::regex_match(v_name.begin(), v_name.end(), rx);
		if (iter->path().filename().extension().string() == ".obj" && bl)
		{
			std::vector<Point3> cornerPoints;

			std::fstream in_offset((myPath / iter->path().stem()).string() + ".txt", std::ios::in);
			std::string offsets;
			in_offset >> offsets;
			int x_offset = atoi(offsets.substr(0, offsets.find(",")).c_str());
			int y_offset = atoi(offsets.substr(offsets.find(",") + 1).c_str());

			SurfaceMesh mesh = cgaltools::convert_obj_from_tinyobjloader_to_surface_mesh(
				modeltools::load_obj(iter->path().string()));
			PointSet3 point_cloud(true);
			for (auto& item_point : mesh.vertices())
			{
				Point3 p(mesh.point(item_point).x() + x_offset, mesh.point(item_point).y() + y_offset,
					mesh.point(item_point).z());
				point_cloud.insert(p);
				mesh.point(item_point) = p;
			}

			// Note: the number will be added by 1 in unreal
			int origin_index = std::atoi(iter->path().stem().string().c_str());
			//int changed_index = origin_index + 1;
			// Do not need to add 1
			int changed_index = origin_index;
			v_out_point_clouds.insert(std::make_pair(std::to_string(changed_index), point_cloud));
			v_out_meshes.insert(std::make_pair(std::to_string(changed_index), mesh));
		}
	}
}

void Virtual_mapper::get_buildings(
	std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
	const int v_cur_frame_id, modeltools::Height_map& v_height_map
)
{
	std::vector<Building> current_buildings;
	int num_building_current_frame = 0;
	// Get current image and pose
	// Input: 
	// Output: Image(cv::Mat), Camera matrix(Pos_pack)
	std::map<std::string, cv::Mat> current_image;
	{
		m_airsim_client->adjust_pose(v_current_pos);
		current_image = m_airsim_client->get_images();
		imwrite("M:\\YRS_debug\\current.jpg", current_image["rgb"]);
		imwrite("M:\\YRS_debug\\seg.jpg", current_image["segmentation"]);
		LOG(INFO) << "Image done";
	}

	// 3D Bounding Box Detection
	// Input: Current_image
	// Output: 3D Bounding Boxes(std::vector<>)
	{
		bool isValid;
		std::vector<CGAL::Bbox_2> boxes_2d;
		std::vector<cv::RotatedRect> boxes_3d;
		std::vector<Point2> zses;
		std::vector<ImageCluster> clusters = solveCluster(current_image["segmentation"], m_color_to_mesh_name_map,
			isValid);
		for (auto& building : clusters)
		{
			Building current_building;
			int index = &building - &clusters[0];
			// Transform the point cloud and meshes from world to camera
			if (!m_meshes.contains(building.name))
				continue;
			SurfaceMesh item_mesh(m_meshes.at(building.name));

			PointSet3 item_points;
			std::copy(item_mesh.points().begin(), item_mesh.points().end(), item_points.point_back_inserter());

			auto box = get_bbox_3d(item_points);

			// 3D Box in mesh coordinate
			Eigen::AlignedBox3d box_3d(Eigen::Vector3d(box.first.center.x - box.first.size.width / 2,
				box.first.center.y - box.first.size.height / 2,
				box.second.x()),
				Eigen::Vector3d(box.first.center.x + box.first.size.width / 2,
					box.first.center.y + box.first.size.height / 2,
					box.second.y()));

			float angle = (-box.first.angle + 90) / 180. * M_PI;

			RotatedBox bounding_box_3d(box_3d, angle);

			current_building.bounding_box_3d = bounding_box_3d;
			current_building.bounding_box_2d = building.box;
			current_buildings.push_back(current_building);
			num_building_current_frame += 1;
		}
	}

	// Merging
	// Input: 3D bounding box of current frame and previous frame
	// Output: Total building vectors (std::vector<Building>)
	{
		float max_iou = 0;
		int max_id = 0;
		std::vector<bool> need_register(num_building_current_frame, false);
		for (auto& item_current_building : current_buildings)
		{
			size_t index_box = &item_current_building - &current_buildings[0];
			for (auto& item_building : v_buildings)
			{
				size_t index_total_box = &item_building - &v_buildings[0];
				float current_iou = calculate_3d_iou(item_current_building, item_building);
				max_iou = std::max(current_iou, max_iou);
				max_id = index_total_box;
			}
			if (max_iou <= m_args["IOU_threshold"].asFloat())
				need_register[index_box] = true;
			else
			{
				item_current_building.passed_trajectory = v_buildings[max_id].passed_trajectory;
				v_buildings[max_id] = item_current_building;
			}
		}
		for (int i = 0; i < need_register.size(); ++i)
		{
			if (need_register[i])
			{
				v_buildings.push_back(current_buildings[i]);
			}
		}
		LOG(INFO) << "Building BBox update: DONE!";
	}

	// Update height map
	for (auto& item_building : v_buildings)
	{
		v_height_map.update(item_building.bounding_box_3d);
	}
}

/*
 * Real Mapper
 */

void Real_mapper::get_buildings(
	std::vector<Building>& v_buildings, const Pos_Pack& v_current_pos,
	const int v_cur_frame_id, modeltools::Height_map& v_height_map
)
{
	std::vector<Building> current_buildings;
	int num_building_current_frame;
	std::vector<cv::KeyPoint> keypoints;
	// Get current image and pose
	// Input: 
	// Output: Image(cv::Mat), Camera matrix(Pos_pack)
	std::map<std::string, cv::Mat> current_image;
	{
		m_airsim_client->adjust_pose(v_current_pos);
		//demo_move_to_next(*(m_airsim_client->m_agent), v_current_pos.pos_airsim, v_current_pos.yaw, 5, false);
		current_image = m_airsim_client->get_images();
		imwrite("F:\\Sig\\Shanghai\\" + std::to_string(v_cur_frame_id) + "_rgb.jpg", current_image["rgb"]);
		imwrite("F:\\Sig\\Shanghai\\" + std::to_string(v_cur_frame_id) + "_seg.png", current_image["segmentation"]);

		LOG(INFO) << "Image done";
	}

	// Object detection
	// Input: Vector of building (std::vector<Building>)
	// Output: Vector of building with 2D bounding box (std::vector<Building>)
	std::vector<cv::Vec3b> color_map;
	std::vector<cv::Rect2f> detection_boxes;
	{
		m_real_object_detector->get_bounding_box(current_image, color_map, current_buildings);
		LOG(INFO) << "Object detection done";
	}

	// SLAM
	// Input: Image(cv::Mat), Camera matrix(cv::Iso)
	// Output: Vector of building with Point cloud in camera frames (std::vector<Building>)
	//		   Refined Camera matrix(cv::Iso)
	//		   num of clusters (int)
	{
		//m_synthetic_SLAM->get_points(current_image, color_map, current_buildings);
		LOG(INFO) << "Sparse point cloud generation and building cluster done";
	}

	cv::Mat rgb = current_image["rgb"].clone();
	auto orb = cv::ORB::create(200);
	//orb->detect(rgb, keypoints, v_img.at("roi_mask"));
	orb->detect(rgb, keypoints);
	drawKeypoints(rgb, keypoints, rgb);
	for (auto box : detection_boxes)
	{
		rectangle(rgb, box, cv::Scalar(0, 0, 255));
	}
	imwrite("F:\\Sig\\demo\\" + std::to_string(v_cur_frame_id) + ".jpg", rgb);


	// Post process point cloud
	// Input: Vector of building (std::vector<Building>)
	// Output: Vector of building with point cloud in world space (std::vector<Building>)
	{
		PointSet3 cur_frame_total_points_in_world_coordinates;
		std::vector<bool> should_delete(current_buildings.size(), false);
		for (auto& item_building : current_buildings)
		{
			size_t cluster_index = &item_building - &current_buildings[0];
			if (item_building.points_camera_space.points().size() < 5)
			{
				should_delete[cluster_index] = true;
				continue;
			}
			for (const auto& item_point : item_building.points_camera_space.points())
			{
				Eigen::Vector3d point_eigen(item_point.x(), item_point.y(), item_point.z());
				point_eigen = v_current_pos.camera_matrix.inverse() * point_eigen;
				item_building.points_world_space.insert(Point3(point_eigen.x(), point_eigen.y(), point_eigen.z()));
				cur_frame_total_points_in_world_coordinates.insert(
					Point3(point_eigen.x(), point_eigen.y(), point_eigen.z()));
			}
			//CGAL::write_ply_point_set(std::ofstream(std::to_string(cluster_index) + "_world.ply"), item_building.points_world_space);
		}
		std::erase_if(current_buildings,
			[&should_delete, idx = 0](const auto& item)mutable
		{
			return should_delete[idx++];
		});
		num_building_current_frame = current_buildings.size();
		//CGAL::write_ply_point_set(std::ofstream(std::to_string(v_cur_frame_id) + "_world_points.ply"), cur_frame_total_points_in_world_coordinates);
	}

	// Mapping
	// Input: *
	// Output: Vector of building with 3D bounding box (std::vector<Building>)
	{
		if (m_args["MAP_2D_BOX_TO_3D"].asBool())
		{
			// Calculate Z distance and get 3D bounding box
			std::vector<float> z_mins(num_building_current_frame, std::numeric_limits<float>::max());
			std::vector<float> z_maxs(num_building_current_frame, std::numeric_limits<float>::min());
			for (const auto& item_building : current_buildings)
			{
				size_t cluster_index = &item_building - &current_buildings[0];
				z_mins[cluster_index] = std::min_element(
					item_building.points_camera_space.range(item_building.points_camera_space.point_map()).begin(),
					item_building.points_camera_space.range(item_building.points_camera_space.point_map()).end(),
					[](const auto& a, const auto& b)
					{
						return a.z() < b.z();
					})->z();
					z_maxs[cluster_index] = std::max_element(
						item_building.points_camera_space.range(item_building.points_camera_space.point_map()).begin(),
						item_building.points_camera_space.range(item_building.points_camera_space.point_map()).end(),
						[](const auto& a, const auto& b)
						{
							return a.z() < b.z();
						})->z();
			}

			// Calculate height of the building, Get 3D bbox world space
			for (auto& item_building : current_buildings)
			{
				size_t cluster_index = &item_building - &current_buildings[0];
				float min_distance = z_mins[cluster_index];
				float max_distance = z_maxs[cluster_index];
				float y_min_2d = item_building.bounding_box_2d.ymin();

				Eigen::Vector3d point_pos_img(0, y_min_2d, 1);
				// BUG HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				//Eigen::Vector3d point_pos_camera_XZ = INTRINSIC.inverse() * point_pos_img;
				Eigen::Vector3d point_pos_camera_XZ = point_pos_img;

				float distance_candidate = min_distance;
				float scale = distance_candidate / point_pos_camera_XZ[2];
				Eigen::Vector3d point_pos_world = v_current_pos.camera_matrix.inverse() * (scale *
					point_pos_camera_XZ);

				float final_height = point_pos_world[2];
				// Shorter than camera, recalculate using max distance
				if (final_height < v_current_pos.pos_mesh[2])
				{
					distance_candidate = max_distance;
					scale = distance_candidate / point_pos_camera_XZ[2];
					point_pos_world = v_current_pos.camera_matrix.inverse() * (scale * point_pos_camera_XZ);
					final_height = point_pos_world[2];
				}

				item_building.bounding_box_3d = cgaltools::get_bounding_box(item_building.points_world_space);
				item_building.bounding_box_3d.box.min()[2] = 0;
				item_building.bounding_box_3d.box.max()[2] = final_height;
			}
		}
		LOG(INFO) << "2D Bbox to 3D Bbox done";
	}

	// Merging
	// Input: 3D bounding box of current frame and previous frame
	// Output: Total building vectors (std::vector<Building>)
	{
		std::vector<bool> need_register(num_building_current_frame, true);
		for (auto& item_building : v_buildings)
		{
			for (const auto& item_current_building : current_buildings)
			{
				// bug here
				/*size_t index_box = &item_current_building - &current_buildings[0];
				if (item_building.segmentation_color == item_current_building.segmentation_color) {
					need_register[index_box] = false;
					item_building.bounding_box_3d = item_building.bounding_box_3d.merged(item_current_building.bounding_box_3d);
					for (const auto& item_point : item_current_building.points_world_space.points())
						item_building.points_world_space.insert(item_point);
				}
				continue;
				if (item_current_building.bounding_box_3d.intersects(item_building.bounding_box_3d)) {
					float overlap_volume = item_current_building.bounding_box_3d.intersection(item_building.bounding_box_3d).volume();
					if (overlap_volume / item_current_building.bounding_box_3d.volume() > 0.5 || overlap_volume / item_building.bounding_box_3d.volume() > 0.5) {
						need_register[index_box] = false;
						item_building.bounding_box_3d = item_building.bounding_box_3d.merged(item_current_building.bounding_box_3d);
						for (const auto& item_point : item_current_building.points_world_space.points())
							item_building.points_world_space.insert(item_point);
					}
				}*/
			}
		}
		for (int i = 0; i < need_register.size(); ++i)
		{
			if (need_register[i])
			{
				v_buildings.push_back(current_buildings[i]);
			}
		}
		LOG(INFO) << "Building BBox update: DONE!";
	}

	// Update height map
	for (auto& item_building : v_buildings)
	{
		v_height_map.update(item_building.bounding_box_3d);
	}
}

/*
 * Unreal object detector
 */

void Unreal_object_detector::get_bounding_box(
	std::map<std::string, cv::Mat>& v_img, std::vector<cv::Vec3b>& v_color_map,
	std::vector<Building>& v_buildings
)
{
	cv::Mat seg = v_img["segmentation"].clone();
	cv::Mat roi_mask(seg.rows, seg.cols, CV_8UC1);
	roi_mask.forEach<uint8_t>(
		[&seg](uint8_t& val, const int* position)
		{
			if (seg.at<cv::Vec3b>(position[0], position[1]) != BACKGROUND_COLOR)
				val = 255;
			else
				val = 0;
		});
	v_img.insert(std::make_pair("roi_mask", roi_mask));
	std::vector<std::vector<cv::Point>> bboxes_points;

	for (int y = 0; y < seg.rows; ++y)
		for (int x = 0; x < seg.cols; ++x)
		{
			if (seg.at<cv::Vec3b>(y, x) == BACKGROUND_COLOR)
				continue;
			bool found = false;
			for (int seg_id = 0; seg_id < v_color_map.size(); ++seg_id)
			{
				if (v_color_map[seg_id] == seg.at<cv::Vec3b>(y, x))
				{
					bboxes_points[seg_id].push_back(cv::Point2f(x, y));
					found = true;
				}
			}
			if (!found)
			{
				v_color_map.push_back(seg.at<cv::Vec3b>(y, x));
				bboxes_points.push_back(std::vector<cv::Point>());
				bboxes_points[v_color_map.size() - 1].push_back(cv::Point2f(x, y));
			}
		}

	v_buildings.resize(v_color_map.size());

	for (const auto& pixel_points : bboxes_points)
	{
		if (pixel_points.size() < 20 * 20)
			continue;
		cv::Rect2f rect = boundingRect(pixel_points);
		rectangle(seg, rect, cv::Scalar(0, 0, 255));
		size_t id = &pixel_points - &*bboxes_points.begin();
		v_buildings[id].bounding_box_2d = CGAL::Bbox_2(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
		v_buildings[id].segmentation_color = v_color_map[id];
	}
	
}

/*
 * Real object detector
 */

std::vector<cv::Rect2f>
Real_object_detector::process_results(
	std::string input, int cols, int rows
)
{
	std::string now_string = input;
	std::vector<std::string> labels;
	std::string::size_type position = now_string.find("], ");
	while (position != now_string.npos)
	{
		labels.push_back(now_string.substr(0, position + 3));
		now_string = now_string.substr(position + 3);
		position = now_string.find("], ");
	}
	if (now_string.length() > 10)
		labels.push_back(now_string);
	std::vector<cv::Rect2f> result;
	for (const auto& label : labels)
	{
		now_string = label;
		cv::Rect2f box;
		std::string::size_type position1 = now_string.find_first_of("[");
		std::string::size_type position2 = now_string.find_first_of(",");
		double xmin = atof((now_string.substr(position1 + 1, position2 - position1 - 1)).c_str());
		now_string = now_string.substr(position2 + 2);
		position2 = now_string.find_first_of(",");
		double ymin = atof((now_string.substr(0, position2 - 1)).c_str());
		now_string = now_string.substr(position2 + 2);
		position2 = now_string.find_first_of(",");
		double xmax = atof((now_string.substr(0, position2 - 1)).c_str());
		now_string = now_string.substr(position2 + 2);
		position2 = now_string.find_first_of("]");
		double ymax = atof((now_string.substr(0, position2 - 1)).c_str());
		if (xmin < 0 && ymin < 0 && xmax > cols && ymax > rows)
			continue;
		if (xmin < 0)
			xmin = 0;
		if (ymin < 0)
			ymin = 0;
		if (xmax > cols)
			xmax = cols;
		if (ymax > rows)
			ymax = rows;
		box.x = xmin;
		box.y = ymin;
		box.width = xmax - xmin;
		box.height = ymax - ymin;
		result.push_back(box);
	}
	return result;
}

cv::Vec3b
Real_object_detector::stringToVec3b(
	std::string input
)
{
	std::string now_string = input;
	std::vector<std::string> color;
	color.push_back(now_string.substr(0, now_string.find_first_of(" ")));
	now_string = now_string.substr(now_string.find_first_of(" ") + 1);
	color.push_back(now_string.substr(0, now_string.find_first_of(" ")));
	now_string = now_string.substr(now_string.find_first_of(" ") + 1);
	color.push_back(now_string);

	return cv::Vec3b(static_cast<uchar>(atoi(color[0].c_str())), static_cast<uchar>(atoi(color[1].c_str())),
		static_cast<uchar>(atoi(color[2].c_str())));
}

void Real_object_detector::get_bounding_box(
	std::map<std::string, cv::Mat>& v_img, std::vector<cv::Vec3b>& v_color_map,
	std::vector<Building>& v_buildings
)
{
	//std::vector<cv::Rect2f> boxes;
	//cv::Mat img = v_img["rgb"];
	//std::vector<uchar> data(img.ptr(), img.ptr() + img.size().width * img.size().height * img.channels());
	//std::string s(data.begin(), data.end());

	//auto r = Post(cpr::Url{ "http://172.31.224.4:10000/index" },
	//	cpr::Body{ s },
	//	cpr::Header{ {"Content-Type", "text/plain"} });
	//std::cout << r.text << std::endl;
	//boxes = process_results(r.text, img.cols, img.rows);
	//// Updata color map
	//for (auto box : boxes)
	//{
	//	// Calculate the main color
	//	std::map<std::string, int> color_num;
	//	cv::Rect rect(box.x, box.y, box.width, box.height);
	//	cv::Mat img_roi = v_img["segmentation"](rect);

	//	for (int y = 0; y < img_roi.rows; y++)
	//	{
	//		for (int x = 0; x < img_roi.cols; x++)
	//		{
	//			auto color = img_roi.at<cv::Vec3b>(y, x);
	//			std::string color_string = Vec3bToString(color);
	//			auto find_result = color_num.find(color_string);
	//			if (find_result == color_num.end())
	//				color_num.insert(std::make_pair(color_string, 1));
	//			else
	//				color_num[color_string] += 1;
	//		}
	//	}
	//	cv::Vec3b current_color;
	//	int max_num = 0;
	//	for (auto color : color_num)
	//	{
	//		if (color.second > max_num)
	//		{
	//			max_num = color.second;
	//			current_color = stringToVec3b(color.first);
	//		}
	//	}
	//	Building current_building;
	//	current_building.bounding_box_2d = CGAL::Bbox_2(box.x, box.y, box.x + box.width, box.y + box.height);
	//	current_building.segmentation_color = current_color;
	//	v_buildings.resize(v_color_map.size());
	//	auto found = std::find(v_color_map.begin(), v_color_map.end(), current_color);
	//	if (found == v_color_map.end())
	//	{
	//		v_color_map.push_back(current_color);
	//		v_buildings.push_back(current_building);
	//	}
	//}
}

void calculate_trajectory_intrinsic(
	const Json::Value& v_args, double& horizontal_step,
	double& vertical_step, double& split_min_distance
)
{
	double view_distance = v_args["view_distance"].asDouble();
	horizontal_step = view_distance *
		std::tan(v_args["fov"].asDouble() / 180. * M_PI / 2) * 2 * (1. - v_args["horizontal_overlap"].asDouble());

	double vertical_reception_field;
	if (v_args["fov"].asFloat() < 60) // If pitch is fixed at 30 degree, then the threshold here is 90-30=60
	{
		double total_part = std::tan((30 + v_args["fov"].asDouble() / 2) / 180. * M_PI) * view_distance;
		double first_part = std::tan((30 - v_args["fov"].asDouble() / 2) / 180. * M_PI) * view_distance;
		vertical_reception_field = total_part - first_part;
	}
	else
	{
		double second_part = std::tan((30 + v_args["fov"].asDouble() / 2) / 180. * M_PI) * view_distance;
		double first_part = std::tan((v_args["fov"].asFloat() / 2 - 30) / 180. * M_PI) * view_distance;
		vertical_reception_field = first_part + second_part;
	}
	if (v_args.isMember("ratio"))
		vertical_reception_field = vertical_reception_field / v_args["ratio"].asDouble();
	vertical_step = vertical_reception_field * (1 - v_args["vertical_overlap"].asDouble());

	split_min_distance = 2 * (std::sqrt(3) - 1) * view_distance - v_args["split_overlap"].asDouble() * (std::sqrt(3) - 1)
		* view_distance; // std::sqrt(3) is the cot(30), 30 is the view angle
	
}