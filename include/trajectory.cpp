#include "trajectory.h"

Eigen::Vector2d lonLat2Mercator(const Eigen::Vector2d& lonLat)
{
	Eigen::Vector2d mercator;
	double x = lonLat.x() * 20037508.34 / 180;
	double y = log(tan((90 + lonLat.y()) * M_PI / 360)) / (M_PI / 180);
	y = y * 20037508.34 / 180;
	mercator = Eigen::Vector2d(x, y);
	return mercator;
}

Eigen::Vector2d mercator2lonLat(const Eigen::Vector2d& mercator)
{
	Eigen::Vector2d lonLat;
	double x = mercator.x() / 20037508.34 * 180;
	double y = mercator.y() / 20037508.34 * 180;
	y = 180 / M_PI * (2 * atan(exp(y * M_PI / 180)) - M_PI / 2);
	lonLat = Eigen::Vector2d(x, y);
	return lonLat;
}

std::vector<Viewpoint> read_wgs84_trajectory(const std::string& v_path, const Eigen::Vector3d& v_original_wgs)
{
	std::vector<Viewpoint> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(" "));

		double longitude = std::atof(tokens[0].c_str());
		double latitude = std::atof(tokens[1].c_str());
		double z = std::atof(tokens[2].c_str());

		Eigen::Vector2d wgs_coordinate(longitude,latitude);
		Eigen::Vector2d mercator_coordinate = lonLat2Mercator(wgs_coordinate);
		Viewpoint viewpoint;
		viewpoint.pos_mesh.x() = mercator_coordinate.x() - v_original_wgs.x();
		viewpoint.pos_mesh.y() = mercator_coordinate.y() - v_original_wgs.y();
		viewpoint.pos_mesh.z() = z - v_original_wgs.z();

		double pitch = -std::atof(tokens[4].c_str() )/ 180. * M_PI;
		double yaw = -(std::atof(tokens[3].c_str()) - 90.)/ 180. * M_PI;

		double dx = std::cos(yaw)*std::cos(pitch);
		double dy = std::sin(yaw)*std::cos(pitch);
		double dz = std::sin(pitch);

		viewpoint.direction = Eigen::Vector3d(dx, dy, dz).normalized();
		o_trajectories.push_back(viewpoint);
	}
	while (!pose.eof());

	pose.close();
	return o_trajectories;
}

std::vector<Viewpoint> read_smith18_path(const std::string& v_path)
{
	std::vector<Viewpoint> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		double x = -std::atof(tokens[1].c_str()) / 100;
		double y = std::atof(tokens[2].c_str()) / 100;
		double z = std::atof(tokens[3].c_str()) / 100;
		
		Viewpoint viewpoint;
		viewpoint.img_name = tokens[0].substr(0,tokens[0].find("."));
		viewpoint.pos_mesh.x() = x;
		viewpoint.pos_mesh.y() = y;
		viewpoint.pos_mesh.z() = z;

		double pitch = -std::atof(tokens[4].c_str() ) / 180. * M_PI;
		double yaw = (90 - std::atof(tokens[6].c_str()) ) / 180. * M_PI;

		double dx = std::cos(yaw)*std::cos(pitch);
		double dy = std::sin(yaw)*std::cos(pitch);
		double dz = std::sin(pitch);

		viewpoint.direction = Eigen::Vector3d(dx, dy, dz).normalized();
		o_trajectories.push_back(viewpoint);
	}
	while (!pose.eof());
	pose.close();
	return o_trajectories;
}

std::vector<Viewpoint> read_normal_path(const std::string& v_path)
{
	std::vector<Viewpoint> o_trajectories;
	std::ifstream pose(v_path);
	if (!pose.is_open()) throw "File not opened";

	std::string line;
	do
	{
		std::getline(pose, line);
		if (line.size() < 3)
		{
			std::getline(pose, line);
			continue;
		}
		std::vector<std::string> tokens;
		boost::split(tokens, line, boost::is_any_of(","));

		double x = std::atof(tokens[1].c_str()) ;
		double y = std::atof(tokens[2].c_str()) ;
		double z = std::atof(tokens[3].c_str());

		Viewpoint viewpoint;
		viewpoint.img_name = tokens[0].substr(0,tokens[0].find("."));
		viewpoint.pos_mesh.x() = x;
		viewpoint.pos_mesh.y() = y;
		viewpoint.pos_mesh.z() = z;

		double pitch = std::atof(tokens[4].c_str() )/ 180. * M_PI;
		double yaw = std::atof(tokens[6].c_str() )/ 180. * M_PI;

		double dx = std::cos(yaw)*std::cos(pitch);
		double dy = std::sin(yaw)*std::cos(pitch);
		double dz = std::sin(pitch);

		viewpoint.direction = Eigen::Vector3d(dx, dy, dz).normalized();
		o_trajectories.push_back(viewpoint);
	}
	while (!pose.eof());
	pose.close();
	return o_trajectories;
}

std::vector<Viewpoint> read_5_camera_pose(const fs::path& v_path)
{
	std::vector<Viewpoint> viewpoints;

	std::ifstream pose(v_path.string());
	if(!pose.is_open()) throw "File not opened";

	std::string line;
	std::vector<std::string> tokens;
	std::getline(pose, line); // title
	std::getline(pose, line);
	while(line.size()>5)
	{
		boost::split(tokens, line, boost::is_any_of(","));
		Eigen::Vector2d wgs_coordinate;
		wgs_coordinate.x() = std::atof(tokens[2].c_str());
		wgs_coordinate.y() = std::atof(tokens[1].c_str());
		double height = std::atof(tokens[4].c_str());
		Eigen::Vector2d mercator_coordinate = lonLat2Mercator(wgs_coordinate);

		double pitch = std::atof(tokens[5].c_str()) / 180. * M_PI;
		double roll = std::atof(tokens[6].c_str()) / 180. * M_PI;
		double yaw = std::atof(tokens[7].c_str()) / 180. * M_PI;;

		Viewpoint viewpoint;
		viewpoint.pos_mesh.x() = mercator_coordinate.x();
		viewpoint.pos_mesh.y() = mercator_coordinate.y();
		viewpoint.pos_mesh.z() = height;

		double dx = std::cos(yaw)*std::cos(pitch);
		double dy = std::sin(yaw)*std::cos(pitch);
		double dz = std::sin(pitch);

		viewpoint.direction = Eigen::Vector3d(dx, dy, dz).normalized();
		viewpoints.push_back(viewpoint);
		tokens.clear();
		std::getline(pose, line);
	}
	
	pose.close();
	
	return viewpoints;
}

std::vector<Viewpoint> read_maya_camera_pose(const fs::path& v_path)
{
	std::vector<Viewpoint> viewpoints;

	std::ifstream pose(v_path.string());
	if(!pose.is_open()) throw "File not opened";

	std::string line;
	std::getline(pose, line); // title
	
	bool newcamera = false;
	bool pos_added = false;

	std::string new_camer_string = "createNode transform -n \"Camera";
	std::string pos_added_string = "setAttr \".translate\" -type \"double3\"";
	std::string	rot_added_string = "setAttr \".rotate\" -type \"double3\"";
	std::string image_path_string = "setAttr -k on \".originalImageFileName\" -type \"string\"";
	double x, y, z;

	while (!pose.eof())
	{
		std::getline(pose, line);

		if (line.rfind(new_camer_string) == 0)
		{
			newcamera = true;
			continue;
		}

		if (line.rfind(pos_added_string) == 1 && newcamera)
		{
			pos_added = true;
			std::vector<std::string> tokens;
			line = line.substr(line.rfind(pos_added_string)+ pos_added_string.size());
			line = line.substr(1, line.size() - 1);
			boost::split(tokens, line, boost::is_any_of(" "));
			
			x = std::atof(tokens[0].c_str());
			y = std::atof(tokens[1].c_str());
			z = std::atof(tokens[2].c_str());
			continue;
		}

		if (line.rfind(rot_added_string) == 1 && newcamera && pos_added)
		{
			std::vector<std::string> tokens;
			line = line.substr(line.rfind(rot_added_string)+ rot_added_string.size());
			line = line.substr(1, line.size() - 1);
			boost::split(tokens, line, boost::is_any_of(" "));

			double heading, pitch, roll;
			pitch = (std::atof(tokens[0].c_str()) - 90) / 180. * M_PI;
			roll = std::atof(tokens[1].c_str()) / 180. * M_PI;
			heading = std::atof(tokens[2].c_str())/180.*M_PI;
			Eigen::Vector3d p_normal(0, 1, 0);

			Eigen::Isometry3d transformation;
			transformation.setIdentity();
			transformation.rotate(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
			transformation.rotate(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX()));
			transformation.rotate(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitY()));

			p_normal = transformation * p_normal;


			pos_added = false;
			newcamera = false;
			Viewpoint viewpoint;
			viewpoint.pos_mesh = Eigen::Vector3d(x, y, z);
			viewpoint.direction = p_normal.normalized();
			
			viewpoints.push_back(viewpoint);
			continue;
		}

		if (line.rfind(image_path_string, 0) == 0)
		{
			//line1 = line1.right(line1.length() - image_path_string.length()).trimmed();
			//line1 = line1.left(line1.length() - 2);
			//line1 = line1.right(line1.length() - 1);
			//QString image_path = line1.trimmed();
			//std::cout << std::string(image_path.toLocal8Bit()) << "\n";
			//tmp.back()->set_image_path(image_path);
		}

	}
	pose.close();
	return viewpoints;
}

std::vector<Viewpoint> read_xml_from_cc_camera_pose(const Eigen::Vector3d v_origin,
	std::vector<std::string>& pose_xml_filenames, bool v_read_sfm_result)
{
	std::vector<Viewpoint> viewpoints;
	std::vector<Eigen::Vector3d> image_unused;
	for (auto& pose_xml_filename : pose_xml_filenames)
	{
		int count_of_photo = 0;
		int count_of_used_photo = 0;
		tinyxml2::XMLDocument doc;
		doc.LoadFile(pose_xml_filename.c_str());
		auto current_array = doc.FirstChildElement("BlocksExchange")->FirstChildElement("Block")->FirstChildElement("Photogroups")->FirstChild();
		while (current_array != nullptr)
		{
			auto current_photo = current_array->FirstChildElement("Photo");
			while (current_photo != nullptr)
			{
				auto pose = current_photo->FirstChildElement("Pose");
				auto rotation = pose->FirstChildElement("Rotation");
				auto center = pose->FirstChildElement("Center");

				if (v_read_sfm_result)
				{
					if(center == nullptr)
					{
						current_photo = current_photo->NextSiblingElement("Photo");
						continue;
					}

					const char* x_text = center->FirstChildElement("x")->GetText();
					const char* y_text = center->FirstChildElement("y")->GetText();
					const char* z_text = center->FirstChildElement("z")->GetText();
					double x = std::atof(x_text);
					double y = std::atof(y_text);

					//这个地方要看xml文件里center是不是经纬度，是经纬度的话就不变，不是的话就注释掉
					//auto final_position = lonLat2Mercator(Eigen::Vector2d(x,y));
					Eigen::Vector2d final_position(x,y);
					final_position.x() -= v_origin.x();
					final_position.y() -= v_origin.y();

					double z = std::atof(z_text);
					z-=v_origin.z();

					double lx = std::atof(rotation->FirstChildElement("M_20")->GetText());
					double ly = std::atof(rotation->FirstChildElement("M_21")->GetText());
					double lz = std::atof(rotation->FirstChildElement("M_22")->GetText());

					count_of_used_photo++;
					if(std::atoi(current_photo->FirstChildElement("Id")->GetText())==0) //289,817
						viewpoints.emplace_back( Eigen::Vector3d(final_position.x(), final_position.y(), z), Eigen::Vector3d(lx,ly,lz).normalized());
				}
				else
				{
					auto center = current_photo->FirstChildElement("ExifData")->FirstChildElement("GPS");
					const char* x_text = center->FirstChildElement("Longitude")->GetText();
					const char* y_text = center->FirstChildElement("Latitude")->GetText();
					const char* z_text = center->FirstChildElement("Altitude")->GetText();
					auto final_position = lonLat2Mercator(Eigen::Vector2d(std::atof(x_text),std::atof(y_text)));
					final_position.x() -= v_origin.x();
					final_position.y() -= v_origin.y();
					double z = atof(z_text);
					z-=v_origin.z();
					Eigen::Vector3d pos(final_position.x(), final_position.y(), z);
					viewpoints.emplace_back(pos,Eigen::Vector3d(0.,0.,-1.));
					image_unused.push_back(pos);
				}

				count_of_photo++;
				current_photo = current_photo->NextSiblingElement("Photo");
			}
			current_array = current_array->NextSibling();
		}
	}
	return viewpoints;
}

void write_wgs_path(const fs::path& v_root, const std::vector<Viewpoint>& v_trajectories,
	const Eigen::Vector3d& v_original_wgs)
{
	Eigen::Vector2d origin_xy(v_original_wgs.x(), v_original_wgs.y());
	std::ofstream pose_total((v_root/"camera_wgs.txt").string());
	std::ofstream pose((v_root/"camera_wgs_0.txt").string());

	Eigen::Vector3d prev_pos = v_trajectories[0].pos_mesh;
	int cur_log_id = 0;
	for (int i_id = 0; i_id < v_trajectories.size(); i_id++) {
		const Eigen::Vector3d& position = v_trajectories[i_id].pos_mesh;
		Eigen::Vector2d pos_mac = Eigen::Vector2d(position.x(), position.y()) + origin_xy;
		Eigen::Vector2d pos_wgs = mercator2lonLat(pos_mac);

		boost::format fmt("%d %d %d %d %d\n");
		double pitch = -std::atan2f(v_trajectories[i_id].direction[2], std::sqrtf(v_trajectories[i_id].direction[0] * v_trajectories[i_id].direction[0] + v_trajectories[i_id].direction[1] * v_trajectories[i_id].direction[1]));
		double yaw = std::atan2f(v_trajectories[i_id].direction[1], v_trajectories[i_id].direction[0]);
		pitch=pitch*180./M_PI;
		yaw=yaw*180./M_PI;

		yaw = -yaw + 90.;
		if (yaw > 180.)
			yaw -= 360.;

		double z = position[2] + v_original_wgs.z();
		pose << (fmt % pos_wgs[0] % pos_wgs[1] % z % yaw % pitch).str();
		pose_total << (fmt % pos_wgs[0] % pos_wgs[1] % z % yaw % pitch).str();
		if((position-prev_pos).norm()>500)
		{
			pose.close();
			pose = std::ofstream((v_root / ("camera_wgs_" + std::to_string(cur_log_id + 1) + ".txt")).string());
			cur_log_id += 1;
		}
		prev_pos = position;
	}
	pose.close();
	pose_total.close();
}

void write_wgs_path_route(const Eigen::Vector3d& v_origin_wgs, const RouteViewpoint& v_trajectories,
	const std::string& v_path)
{
	
}

void write_smith18_path(const std::vector<Viewpoint>& v_trajectories, const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i)
	{
		const Eigen::Vector3d& position = v_trajectories[i].pos_mesh * 100;
		const Eigen::Vector3d& direction = v_trajectories[i].direction;
		boost::format fmt("%04d.png,%s,%s,%s,%s,0,%s\n");
		double pitch = std::atan2f(direction[2], std::sqrtf(direction[0] * direction[0] + direction[1] * direction[1])) *
			180. / M_PI;
		double yaw = std::atan2f(direction[1], direction[0]) * 180. / M_PI;
		yaw = 90-yaw;
		pose << (fmt % i % -position[0] % position[1] % position[2] % -pitch % yaw).str();
	}

	pose.close();
}

void write_normal_path(const std::vector<Viewpoint>& v_trajectories, const std::string& v_path)
{
	std::ofstream pose(v_path);
	for (int i = 0; i < v_trajectories.size(); ++i)
	{
		const Eigen::Vector3d& position = v_trajectories[i].pos_mesh;
		boost::format fmt("%04d,%s,%s,%s,%s,0,%s,\n");

		double pitch = std::asin(v_trajectories[i].direction.z()) * 180. / M_PI;
		double yaw = std::atan2(v_trajectories[i].direction.y(),v_trajectories[i].direction.x()) * 180. / M_PI;

		pose << (fmt % i % position[0] % position[1] % position[2] % pitch % yaw).str();
	}

	pose.close();
}
