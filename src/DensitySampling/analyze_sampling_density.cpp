#include <iostream>
#include <boost/filesystem.hpp>
#include <argparse/argparse.hpp>
#include <json/reader.h>
#include <glog/logging.h>

#include "common_util.h"
#include "metrics.h"
#include "viz.h"
#include "trajectory.h"

#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Search_traits_adapter.h>


int main(int argc, char** argv){
    std::string log_root("../../../log/DensitySampling/");

	std::cout << "Read config " << argv[2] << std::endl;
	Json::Value args;
	argparse::ArgumentParser program("Analyze");
	{
		program.add_argument("--model").required();
		program.add_argument("--density_important").required();
		program.add_argument("--density_unimportant").required();
		program.add_argument("--filter_z").default_value(0);
		try {
			program.parse_args(argc, argv);
		}
		catch (const std::runtime_error& err) {
			std::cout << err.what() << std::endl;
			std::cout << program;
			exit(0);
		}
	}
	google::InitGoogleLogging(argv[0]);
	google::SetStderrLogging(google::GLOG_INFO);

	LOG(INFO) << "Loading model";
	const std::string model_file = program.get<std::string>("--model");
	const double density_important = std::atof(program.get<std::string>("--density_important").c_str());
	const double density_unimportant = std::atof(program.get<std::string>("--density_unimportant").c_str());
	const double filter_z = std::atof(program.get<std::string>("--filter_z").c_str());

	SurfaceMesh model_important,model_unimportant;
	
	model_important = modeltools::read_model(model_file);
	CGAL::IO::write_PLY("test.ply",model_important);
	LOG(INFO) << boost::format("%d vertices, %d faces") % model_important.num_vertices() % model_important.num_faces();
	model_unimportant = model_important;

	for (auto face : model_important.faces())
	{
		bool remove = true;
		for (auto vertex : model_important.vertices_around_face(model_important.halfedge(face)))
		{
			if (model_important.point(vertex).z() > filter_z)
				remove = false;
		}
		if (remove)
			CGAL::remove_face(face, model_important);
	}
	for (auto face : model_unimportant.faces())
	{
		bool remove = true;
		for (auto vertex : model_unimportant.vertices_around_face(model_unimportant.halfedge(face)))
		{
			if (model_unimportant.point(vertex).z() > filter_z)
				remove = false;
		}
		if (!remove)
			CGAL::remove_face(face, model_unimportant);
	}

	CGAL::collect_garbage(model_important);
	CGAL::collect_garbage(model_unimportant);
	CGAL::IO::write_PLY(log_root + "model_important.ply", model_important);
	CGAL::IO::write_PLY(log_root + "model_unimportant.ply", model_unimportant);


	double important_area = CGAL::Polygon_mesh_processing::area(model_important);
	double unimportant_area = CGAL::Polygon_mesh_processing::area(model_unimportant);
	double total_area = important_area + unimportant_area;

	int num_important_sample = important_area * density_important;
	int num_unimportant_sample = unimportant_area * density_unimportant;
	PointSet3 important_sample_points = modeltools::sample_points(model_important, num_important_sample);
	PointSet3 unimportant_sample_points = modeltools::sample_points(model_unimportant, num_unimportant_sample);
	CGAL::IO::write_point_set(log_root + "points_important.ply", important_sample_points);
	CGAL::IO::write_point_set(log_root + "points_unimportant.ply", unimportant_sample_points);

	PointSet3 whole_points(important_sample_points);
	for (auto point : unimportant_sample_points)
		whole_points.insert(unimportant_sample_points.point(point), unimportant_sample_points.normal(point));
	CGAL::IO::write_point_set(log_root + "points_whole.ply", whole_points);

	LOG(INFO) << boost::format("Important part: %d points; Unimportant part: %d points") % important_sample_points.size() % unimportant_sample_points.size();

	My_Visualizer viz(model_file);
	viz.lock();
	viz.m_important_points = important_sample_points;
	viz.m_unimportant_points = unimportant_sample_points;
	viz.unlock();

    comutil::debug_img();
}