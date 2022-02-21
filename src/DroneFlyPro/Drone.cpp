// Drone.cpp : Defines the entry point for the application.

#include "Drone.h"

using namespace std;

string configurationPath = "C:/SSD/GitProject/MarinaioDrone/conf/conf1.json";

int main() {

	json args;
	fs::path log_root("log");
	try
	{
		std::ifstream in(configurationPath);
		if (!in.is_open())
		{
			LOG(ERROR) << "Error opening file " << configurationPath;
			return 1;
		}
		in >> args;
		in.close();
	}
	catch (const std::runtime_error& err)
	{
		std::cout << err.what() << std::endl;
		exit(1);
	}

	comutil::checkFolder(log_root);
	google::InitGoogleLogging(args["project"].get<string>().c_str());
	google::SetStderrLogging(google::GLOG_INFO);
	google::SetLogDestination(google::GLOG_INFO, log_root.string().c_str());


	PointSet3 pts;
	CGAL::IO::read_point_set(args["plyModelPath"], pts);
	auto obj = modeltools::load_obj(args["objModelPath"]);
	auto surf = cgaltools::convert_obj_from_tinyobjloader_to_surface_mesh(obj);
	intersectiontools::remove_points_inside(surf, pts);
	using Point_with_normal = std::tuple<Point3, Vector3>;
	std::ofstream ofile("C:/Marinaio/Data/nn.ply");
	CGAL::IO::set_binary_mode(ofile);
	CGAL::IO::write_PLY(ofile, pts);


	std::cout<< "WECANBOTGOBACK" << std::endl;
	return 0;
}