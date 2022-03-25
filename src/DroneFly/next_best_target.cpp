#include "next_best_target.h"

Next_best_target_topology_exploration::Next_best_target_topology_exploration(
	const Eigen::Vector3d& v_map_start_mesh, const Eigen::Vector3d& v_map_end_mesh,
	int v_CCPP_CELL_THRESHOLD, const Polygon2& v_boundary, float v_ccpp_cell_distance, const Json::Value& v_arg
) :CCPP_CELL_THRESHOLD(v_CCPP_CELL_THRESHOLD), m_arg(v_arg), Next_best_target(v_map_start_mesh, v_map_end_mesh, v_ccpp_cell_distance)
{
	color_reconstruction = cv::Vec3b(0, 255, 0);
	color_occupied = cv::Vec3b(0, 255, 0);
	color_unobserved = cv::Vec3b(205, 205, 209);
	this->m_boundary = v_boundary;
	region_status.clear();
	region_status.resize(sample_points.size(), color_unobserved);

	if (v_boundary.size() != 0) {
		for (int i_sample_point = 0; i_sample_point < sample_points.size(); ++i_sample_point) {
			if (v_boundary.bounded_side(sample_points[i_sample_point]) != CGAL::ON_BOUNDED_SIDE) {
				//LOG(INFO) << v_boundary.is_simple();
				region_status[i_sample_point] = color_occupied;
			}
		}
	}

	m_current_color_id = 0;
	m_current_ccpp_trajectory_id = 0;
}

void
Next_best_target_topology_exploration::get_next_target(
	int frame_id, const Pos_Pack& v_cur_pos, 
	const std::vector<Building>& v_buildings, bool with_exploration
) 
{
	m_ccpp_trajectory.clear();
	m_current_ccpp_trajectory_id = -1;
	// Find next target (building or place) with higher confidence
	std::vector<int> untraveled_buildings;
	for (int i_building = 0; i_building < v_buildings.size(); ++i_building) {
		if (v_buildings[i_building].is_divide)
			continue;
		else if (v_buildings[i_building].passed_trajectory.size() == 0)
		{
			untraveled_buildings.push_back(i_building);
		}
	}
	if (untraveled_buildings.size() == 0)
	{
		get_ccpp_trajectory(v_cur_pos.pos_mesh,
			default_expand(), 0);
		return;
	}

	// Find nearest building to existing polygon
	std::sort(untraveled_buildings.begin(),
		untraveled_buildings.end(),
		[&v_cur_pos, &v_buildings, this](const int& b1, const int& b2) {
			int view1 = v_buildings[b1].find_nearest_trajectory_2d(v_cur_pos.pos_mesh);
			int view2 = v_buildings[b2].find_nearest_trajectory_2d(v_cur_pos.pos_mesh);
			return (v_buildings[b1].trajectory[view1].pos_mesh - v_cur_pos.pos_mesh).norm() < (v_buildings[b2].trajectory[view2].pos_mesh - v_cur_pos.pos_mesh).norm();
			//return (v_buildings[id1].trajectory[view1].first.x()- v_cur_pos.pos_mesh.x()) < (v_buildings[id2].trajectory[view2].first.x() - v_cur_pos.pos_mesh.x());
		});
	//std::sort(untraveled_buildings.begin(),
	//	untraveled_buildings.end(),
	//	[&v_cur_pos, &v_buildings, this](const int& b1, const int& b2) {
	//		return (v_buildings[b1].bounding_box_3d.box.center() - v_cur_pos.pos_mesh).norm() < (v_buildings[b2].bounding_box_3d.box.center() - v_cur_pos.pos_mesh).norm();
	//		//return (v_buildings[id1].trajectory[view1].first.x()- v_cur_pos.pos_mesh.x()) < (v_buildings[id2].trajectory[view2].first.x() - v_cur_pos.pos_mesh.x());
	//});

	int id_building = 0;
	if (with_exploration)
	{
		bool ccpp_done = get_ccpp_trajectory(v_cur_pos.pos_mesh, v_buildings[untraveled_buildings[id_building]],
			CCPP_CELL_THRESHOLD);
		while (!ccpp_done)
		{
			id_building += 1;
			if (id_building >= untraveled_buildings.size())
			{
				get_ccpp_trajectory(v_cur_pos.pos_mesh,
					default_expand(), 0);
				return;
			}
			else
				ccpp_done = get_ccpp_trajectory(v_cur_pos.pos_mesh, v_buildings[untraveled_buildings[id_building]],
					CCPP_CELL_THRESHOLD);
		}
	}

	// return trajectory
	//debug_img(std::vector<cv::Mat>{map});
	m_current_building_id = untraveled_buildings[id_building];
	if (with_exploration)
		m_motion_status = Motion_status::exploration;
	else
		m_motion_status = Motion_status::reconstruction_in_exploration;
	return;
}

void
Next_best_target_topology_exploration::update_uncertainty(
	const Pos_Pack& v_cur_pos, const std::vector<Building>& v_buildings) 
{
	Eigen::Vector2d cur_point_cgal(v_cur_pos.pos_mesh.x(), v_cur_pos.pos_mesh.y());
	int nearest_region_id = std::min_element(sample_points.begin(), sample_points.end(),
		[&cur_point_cgal](const CGAL::Point_2<K>& p1, const CGAL::Point_2<K>& p2) {
			return std::pow(p1.x() - cur_point_cgal.x(), 2) + std::pow(p1.y() - cur_point_cgal.y(), 2) < std::pow(p2.x() - cur_point_cgal.x(), 2) + std::pow(p2.y() - cur_point_cgal.y(), 2);
		}) - sample_points.begin();
		//if (region_status[nearest_region_id] == color_unobserved)
		//{
			//if (m_motion_status == Motion_status::reconstruction)
			//	region_status[nearest_region_id] = color_reconstruction;
			//else
		//		region_status[nearest_region_id] = region_viz_color[m_current_color_id % (region_viz_color.size() - 2) + 2];
		//}
		if ((m_motion_status == Motion_status::exploration || m_motion_status == Motion_status::final_check) && region_status[nearest_region_id] == color_unobserved)
		{
			bool inside = false;
			for (auto item : topology)
				if (modeltools::inside_box(Eigen::Vector2d(sample_points[nearest_region_id].x(), sample_points[nearest_region_id].y()), item))
					inside = true;

			if (inside && m_boundary.bounded_side(sample_points[nearest_region_id]) == CGAL::ON_BOUNDED_SIDE)
				region_status[nearest_region_id] = region_viz_color[m_current_color_id % region_viz_color.size()];
		}

		for (int i_point = 0; i_point < region_status.size(); i_point++) {
			const Eigen::Vector2d p(sample_points[i_point].x(), sample_points[i_point].y());

			for (const auto& item_building : v_buildings) {

				//Eigen::AlignedBox2d box(Eigen::Vector2d(item_building.bounding_box_3d.min().x(), item_building.bounding_box_3d.min().y()),
				//	Eigen::Vector2d(item_building.bounding_box_3d.max().x(), item_building.bounding_box_3d.max().y()));
				//if (point_box_distance_eigen(p, box) < 20 || inside_box(p, box)) {
				//if (inside_box(p, box)) {
					//region_status[i_point] = color_occupied;
				//	break;
				//}
			}
		}
}

MyViewpoint
Next_best_target_topology_exploration::determine_next_target(
	int v_frame_id, const Pos_Pack& v_cur_pos, std::vector<Building>& v_buildings,
	bool with_exploration, float v_threshold
)
{
	bool with_reconstruction = m_arg["with_reconstruction"].asBool();

	MyViewpoint next_pos;
	if (m_motion_status == Motion_status::initialization) {
		get_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration);
		LOG(INFO) << "Initialization target !";
	}
	if (m_motion_status == Motion_status::exploration || m_motion_status == Motion_status::final_check)
	{
		if (with_exploration == false)
		{
			if (m_motion_status == Motion_status::final_check)
			{
				m_motion_status = Motion_status::done;
				return next_pos;
			}
			else
			{
				m_motion_status = Motion_status::reconstruction_in_exploration;
				next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				return next_pos;
			}
		}
		std::vector<int> untraveled_buildings_inside_exist_region;
		Eigen::Vector2d cur_point_cgal(m_ccpp_trajectory[m_current_ccpp_trajectory_id].pos_mesh.x(),
			m_ccpp_trajectory[m_current_ccpp_trajectory_id].pos_mesh.y());
		for (int i_building = 0; i_building < v_buildings.size(); ++i_building) {
			if (v_buildings[i_building].is_divide)
				continue;
			if (v_buildings[i_building].passed_trajectory.size() == 0) {
				//int id_nearest_view = v_buildings[i_building].find_nearest_trajectory_2d(Eigen::Vector3d(cur_point_cgal.x(), cur_point_cgal.y(), 0));
				//Eigen::Vector2d nearest_view(v_buildings[i_building].trajectory[id_nearest_view].first.x(), 
				//	v_buildings[i_building].trajectory[id_nearest_view].first.y());
				Eigen::Vector2d nearest_view(v_buildings[i_building].bounding_box_3d.box.center().x(),
					v_buildings[i_building].bounding_box_3d.box.center().y());

				if ((nearest_view - cur_point_cgal).norm() < DISTANCE_THRESHOLD / 1.4)
					untraveled_buildings_inside_exist_region.push_back(i_building);
			}
		}
		if (untraveled_buildings_inside_exist_region.size() != 0 && !m_isFirst) {
			//throw;
			m_motion_status = Motion_status::reconstruction_in_exploration;
			int id_building = std::min_element(untraveled_buildings_inside_exist_region.begin(),
				untraveled_buildings_inside_exist_region.end(),
				[&v_cur_pos, &v_buildings, this](const int& id1, const int& id2) {
					int view1 = v_buildings[id1].find_nearest_trajectory_2d(v_cur_pos.pos_mesh);
					int view2 = v_buildings[id2].find_nearest_trajectory_2d(v_cur_pos.pos_mesh);
					return (v_buildings[id1].trajectory[view1].pos_mesh - v_cur_pos.pos_mesh).norm() < (v_buildings[id2].trajectory[view2].pos_mesh - v_cur_pos.pos_mesh).norm();
					//return (v_buildings[id1].trajectory[view1].first.x()- v_cur_pos.pos_mesh.x()) < (v_buildings[id2].trajectory[view2].first.x() - v_cur_pos.pos_mesh.x());
				}) - untraveled_buildings_inside_exist_region.begin();
				m_current_building_id = untraveled_buildings_inside_exist_region[id_building];
				next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				return next_pos;
		}
		else
		{
			if (m_exploration_point.size() == 0)
			{
				m_current_ccpp_trajectory_id += 1;
				if (m_current_ccpp_trajectory_id >= m_ccpp_trajectory.size()) {
					if (m_motion_status == Motion_status::final_check) {
						m_motion_status = Motion_status::done;
						LOG(INFO) << dummy2;
						LOG(INFO) << dummy3;
					}
					else {
						get_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration);
						m_isFirst = true;
						next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
						m_isFirst = false;
						//m_motion_status = Motion_status::reconstruction;
						m_current_color_id += 1;
						return next_pos;
					}
				}
				else {
					auto& item = m_ccpp_trajectory[m_current_ccpp_trajectory_id].pos_mesh;

					std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> start_points;

					start_points.emplace_back(
						Eigen::Vector3d(
							item.x() - DISTANCE_THRESHOLD * 0.25,
							item.y(),
							100),
						Eigen::Vector3d(1, 0, 0)
					);
					start_points.emplace_back(
						Eigen::Vector3d(
							item.x() + DISTANCE_THRESHOLD * 0.25,
							item.y(),
							100),
						Eigen::Vector3d(-1, 0, 0)
					);
					start_points.emplace_back(
						Eigen::Vector3d(
							item.x(),
							item.y() + DISTANCE_THRESHOLD * 0.25,
							100),
						Eigen::Vector3d(0, -1, 0)
					);
					start_points.emplace_back(
						Eigen::Vector3d(
							item.x(),
							item.y() - DISTANCE_THRESHOLD * 0.25,
							100),
						Eigen::Vector3d(0, 1, 0)
					);

					int nearest_id = std::min_element(start_points.begin(), start_points.end(),
						[&v_cur_pos](auto& a1, auto& a2) {
							return (v_cur_pos.pos_mesh - a1.first).norm() < (v_cur_pos.pos_mesh - a2.first).norm();
						}) - start_points.begin();

						if (m_arg["explore_multi_angle"].asBool())
							m_exploration_point.emplace(
								start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 0 * start_points[nearest_id].second,
								start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 0 * start_points[nearest_id].second +
								(m_arg["fix_angle_flag"].asBool() ? Eigen::Vector3d(std::cos(64.f / 180 * M_PI), 0, -std::sin(64.f / 180 * M_PI)) : Eigen::Vector3d(-0.5, -0.5 * 1.732, -std::tan(64.f / 180 * M_PI)).normalized())
							);
						m_exploration_point.emplace(
							start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 1 * start_points[nearest_id].second,
							start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 1 * start_points[nearest_id].second +
							(m_arg["fix_angle_flag"].asBool() ? Eigen::Vector3d(std::cos(64.f / 180 * M_PI), 0, -std::sin(64.f / 180 * M_PI)) : Eigen::Vector3d(1, 0, -std::tan(64.f / 180 * M_PI)).normalized())
						);
						if (m_arg["explore_multi_angle"].asBool())
							m_exploration_point.emplace(
								start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 2 * start_points[nearest_id].second,
								start_points[nearest_id].first + DISTANCE_THRESHOLD * 0.25 * 2 * start_points[nearest_id].second +
								(m_arg["fix_angle_flag"].asBool() ? Eigen::Vector3d(std::cos(64.f / 180 * M_PI), 0, -std::sin(64.f / 180 * M_PI)) : Eigen::Vector3d(-0.5, 0.5 * 1.732, -std::tan(64.f / 180 * M_PI)).normalized())
							);


						/*m_exploration_point.emplace(
							Eigen::Vector3d(
								item.x() - DISTANCE_THRESHOLD * 0.3,
								item.y(),
								100),
							Eigen::Vector3d(0, 1, -std::tan(64.f / 180 * M_PI)).normalized()
						);
						m_exploration_point.emplace(
							Eigen::Vector3d(
								item.x() - DISTANCE_THRESHOLD * 0.1,
								item.y(),
								100),
							Eigen::Vector3d(1, 0, -std::tan(64.f / 180 * M_PI)).normalized()
						);
						m_exploration_point.emplace(
							Eigen::Vector3d(
								item.x() + DISTANCE_THRESHOLD * 0.1,
								item.y(),
								100),
							Eigen::Vector3d(0, -1, -std::tan(64.f / 180 * M_PI)).normalized()
						);
						m_exploration_point.emplace(
							Eigen::Vector3d(
								item.x() + DISTANCE_THRESHOLD * 0.3,
								item.y(),
								100),
							Eigen::Vector3d(-1, 0, -std::tan(64.f / 180 * M_PI)).normalized()
						); */
						//m_exploration_point.emplace(
						//	Eigen::Vector3d(
						//		item.x(),
						//		item.y(),
						//		100),
						//	Eigen::Vector3d(-1, 0, -std::tan(64.f / 180 * M_PI)).normalized()
						//);
						//next_pos = m_ccpp_trajectory[m_current_ccpp_trajectory_id];
						next_pos = m_exploration_point.front();
						next_pos.is_towards_reconstruction = false;
						m_exploration_point.pop();
				}
			}
			else
			{
				auto item = m_exploration_point.front();
				m_exploration_point.pop();
				item.is_towards_reconstruction = false;
				return item;
			}
			//m_motion_status = Motion_status::exploration;

		}
	}
	if (m_motion_status == Motion_status::reconstruction_in_exploration)
	{
		if (!with_reconstruction)
		{
			if (with_exploration)
			{
				m_motion_status = Motion_status::exploration;
				next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				next_pos.is_towards_reconstruction = false;
			}
			else
				throw;
		}
		if (true)
		{
			const int& cur_building_id = m_current_building_id;
			std::vector<MyViewpoint> unpassed_trajectory;
			std::vector<MyViewpoint>& passed_trajectory = v_buildings[cur_building_id].passed_trajectory;
			if (passed_trajectory.size() == 0)
			{
				int id_closest_trajectory = std::min_element(v_buildings[cur_building_id].trajectory.begin(), v_buildings[cur_building_id].trajectory.end(),
					[&v_cur_pos](const auto& tra1, const auto& tra2) {
						return (tra1.pos_mesh - v_cur_pos.pos_mesh).norm() < (tra2.pos_mesh - v_cur_pos.pos_mesh).norm();
					}) - v_buildings[cur_building_id].trajectory.begin();
					v_buildings[cur_building_id].closest_trajectory_id = id_closest_trajectory;
			}
			int start_pos_id = 0;
			int one_pass_trajectory_num = v_buildings[cur_building_id].one_pass_trajectory_num;
			std::copy_if(v_buildings[cur_building_id].trajectory.begin() + v_buildings[cur_building_id].closest_trajectory_id, v_buildings[cur_building_id].trajectory.end(),
				std::back_inserter(unpassed_trajectory),
				[&passed_trajectory, v_threshold, &unpassed_trajectory, &start_pos_id, one_pass_trajectory_num](const MyViewpoint& item_new_trajectory) {
					bool untraveled = true;
					for (auto item_passed_trajectory_iter = passed_trajectory.begin(); item_passed_trajectory_iter < passed_trajectory.end(); ++item_passed_trajectory_iter) {
						auto item_passed_trajectory = *item_passed_trajectory_iter;
						Eigen::Vector3d vector1 = item_passed_trajectory.direction;
						Eigen::Vector3d vector2 = item_new_trajectory.direction;
						float dot_product = vector1.dot(vector2);
						if (dot_product > 1)
							dot_product = 1;
						float angle = std::acos(dot_product) / M_PI * 180;
						if (angle >= 180)
							angle = 0;
						if ((item_passed_trajectory.pos_mesh - item_new_trajectory.pos_mesh).norm() < v_threshold && angle < 0.1) {
							untraveled = false;
							/*if (unpassed_trajectory.size() - start_pos_id < one_pass_trajectory_num / 2)
								start_pos_id = unpassed_trajectory.size();
							if ((item_passed_trajectory_iter - passed_trajectory.begin()) == passed_trajectory.size() - 1)
								start_pos_id = 0;*/
								//TODO trajectory merge.
						}
					}
					return untraveled;
				});

			std::copy_if(v_buildings[cur_building_id].trajectory.begin(), v_buildings[cur_building_id].trajectory.begin() + v_buildings[cur_building_id].closest_trajectory_id,
				std::back_inserter(unpassed_trajectory),
				[&passed_trajectory, v_threshold, &unpassed_trajectory, &start_pos_id, one_pass_trajectory_num](const MyViewpoint& item_new_trajectory) {
					bool untraveled = true;
					for (auto item_passed_trajectory_iter = passed_trajectory.begin(); item_passed_trajectory_iter < passed_trajectory.end(); ++item_passed_trajectory_iter) {
						auto item_passed_trajectory = *item_passed_trajectory_iter;
						Eigen::Vector3d vector1 = item_passed_trajectory.direction;
						Eigen::Vector3d vector2 = item_new_trajectory.direction;
						float dot_product = vector1.dot(vector2);
						if (dot_product > 1)
							dot_product = 1;
						float angle = std::acos(dot_product) / M_PI * 180;
						if (angle >= 180)
							angle = 0;
						if ((item_passed_trajectory.pos_mesh - item_new_trajectory.pos_mesh).norm() < v_threshold && angle < 0.1) {
							untraveled = false;
							/*if (unpassed_trajectory.size() - start_pos_id < one_pass_trajectory_num / 2)
								start_pos_id = unpassed_trajectory.size();
							if ((item_passed_trajectory_iter - passed_trajectory.begin()) == passed_trajectory.size() - 1)
								start_pos_id = 0;*/
								//TODO trajectory merge.
						}
					}
					return untraveled;
				});

			if (unpassed_trajectory.size() == 0) {
				//debug_img(std::vector<cv::Mat>{cv::Mat(1, 1, CV_8UC1)});
				if (with_exploration)
				{
					m_motion_status = Motion_status::exploration;
					next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				}
				else
				{
					m_motion_status = Motion_status::reconstruction_in_exploration;
					get_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration);
					if (m_motion_status == Motion_status::final_check)
					{
						m_motion_status = Motion_status::done;
						return next_pos;
					}

					next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				}
				next_pos.is_towards_reconstruction = false;

				//get_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration);
				//LOG(INFO) << "Change target !";
				//return determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
			}
			else {
				if (start_pos_id >= unpassed_trajectory.size())
					start_pos_id = 0;
				next_pos = unpassed_trajectory.at(start_pos_id);
				passed_trajectory.push_back(next_pos);
				next_pos.is_towards_reconstruction = false;

				return next_pos;
			}
		}
		else
		{
			const int& cur_building_id = m_current_building_id;
			std::vector<MyViewpoint>& passed_trajectory = v_buildings[cur_building_id].passed_trajectory;
			if (passed_trajectory.size() == 0)
			{
				int id_closest_trajectory = std::min_element(v_buildings[cur_building_id].trajectory.begin(), v_buildings[cur_building_id].trajectory.end(),
					[&v_cur_pos](const auto& tra1, const auto& tra2) {
						return (tra1.pos_mesh - v_cur_pos.pos_mesh).norm() < (tra2.pos_mesh - v_cur_pos.pos_mesh).norm();
					}) - v_buildings[cur_building_id].trajectory.begin();
					v_buildings[cur_building_id].closest_trajectory_id = id_closest_trajectory;
			}

			if (passed_trajectory.size() == v_buildings[cur_building_id].trajectory.size()) // Exit reconstruction mode
			{
				if (with_exploration)
				{
					m_motion_status = Motion_status::exploration;
					next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				}
				else
				{
					m_motion_status = Motion_status::reconstruction_in_exploration;
					get_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration);
					if (m_motion_status == Motion_status::final_check)
					{
						m_motion_status = Motion_status::done;
						return next_pos;
					}

					next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
				}
			}
			else
			{
				int id = v_buildings[cur_building_id].closest_trajectory_id + passed_trajectory.size();
				if (id >= v_buildings[cur_building_id].trajectory.size())
					id -= v_buildings[cur_building_id].trajectory.size();
				next_pos = v_buildings[cur_building_id].trajectory[id];
				passed_trajectory.push_back(next_pos);
				return next_pos;

			}
		}

		//const int& cur_building_id = m_current_building_id;
		//std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> unpassed_trajectory;
		//std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>>& passed_trajectory = v_buildings[cur_building_id].passed_trajectory;
		//std::copy_if(v_buildings[cur_building_id].trajectory.begin(), v_buildings[cur_building_id].trajectory.end(),
		//	std::back_inserter(unpassed_trajectory),
		//	[&passed_trajectory, v_threshold](const auto& item_new_trajectory) {
		//	bool untraveled = true;
		//	for (const auto& item_passed_trajectory : passed_trajectory)
		//		if ((item_passed_trajectory.first - item_new_trajectory.first).norm() < v_threshold) {
		//			untraveled = false;
		//		}
		//	return untraveled;
		//});

		//if (unpassed_trajectory.size() == 0) {
		//	//debug_img(std::vector<cv::Mat>{cv::Mat(1,1,CV_8UC1)});
		//	m_motion_status = Motion_status::exploration;
		//	next_pos = determine_next_target(v_frame_id, v_cur_pos, v_buildings, with_exploration, v_threshold);
		//}
		//else
		//{
		//	auto it_min_distance = std::min_element(
		//		unpassed_trajectory.begin(), unpassed_trajectory.end(),
		//		[&v_cur_pos](const auto& t1, const auto& t2) {
		//		return (t1.first - v_cur_pos.pos_mesh).norm() < (t2.first - v_cur_pos.pos_mesh).norm();
		//	});
		//	next_pos = *it_min_distance;
		//	passed_trajectory.push_back(next_pos);
		//}
	}
	return next_pos;
}