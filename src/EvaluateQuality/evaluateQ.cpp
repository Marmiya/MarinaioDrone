#define TINYOBJLOADER_IMPLEMENTATION 

#include <iostream>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/OpenGR/compute_registration_transformation.h>
#include <CGAL/IO/write_ply_points.h>
#include <CGAL/polygon_mesh_processing.h>
#include <CGAL/point_set_processing_assertions.h>
#include <CGAL/aff_transformation_tags.h>

#include <json/json.h>
#include <glog/logging.h>
#include <argparse/argparse.hpp>

#include "common_util.h"
#include "intersection_tools.h"
#include "icp.h"
#include "model_tools.h"

typedef std::vector<Triangle3>::iterator Triangle_Iterator;
typedef CGAL::AABB_triangle_primitive<K, Triangle_Iterator> Triangle_Primitive;
typedef CGAL::AABB_traits<K, Triangle_Primitive> Triangle_AABB_triangle_traits;
typedef CGAL::AABB_tree<Triangle_AABB_triangle_traits> Triangle_Tree;


int main(int argc, char** argv){
	bool interaction_on;
	boost::filesystem::path log_root("log");
	if(argv[2]=="")
	{
		std::cout << "Please specify a config file";
		return 0;
	}
	std::cout << "Read config " << argv[2] << std::endl;
	Json::Value args;
	{
		google::InitGoogleLogging(argv[0]);
		argparse::ArgumentParser program("Jointly exploration, navigation and reconstruction");
		program.add_argument("--config_file").required();
		program.add_argument("--recon_model");
		program.add_argument("--gt_model");
		program.add_argument("--viz");
		program.add_argument("--local_icp");
		try {
			program.parse_args(argc, argv);
			const std::string config_file = program.get<std::string>("--config_file");

			std::ifstream in(config_file);
			if (!in.is_open()) {
				LOG(ERROR) << "Error opening file" << config_file << std::endl;
				return 0;
			}
			Json::Reader json_reader;
			if (!json_reader.parse(in, args)) {
				LOG(ERROR) << "Error parse config file" << config_file << std::endl;
				return 0;
			}
			in.close();

			if (auto fn = program.present("--recon_model"))
				args["recon_model"] = program.get<std::string>("--recon_model");
			if (auto fn = program.present("--gt_model"))
				args["gt_model"] = program.get<std::string>("--gt_model");
			if (auto fn = program.present("--local_icp"))
				args["local_icp"] = std::atoi(program.get<std::string>("--local_icp").c_str());
			if (auto fn = program.present("--viz"))
				interaction_on = std::atoi(program.get<std::string>("--viz").c_str());
		}
		catch (const std::runtime_error& err) {
			std::cout << err.what() << std::endl;
			std::cout << program;
			exit(0);
		}
	}
	FLAGS_logtostderr = true;

	CGAL::Aff_transformation_3<K> pre_transformation=CGAL::Identity_transformation();
	if(args.isMember("matrix"))
	{
		if(args["matrix"].size()!=12)
		{
			LOG(INFO) << "Matrix should be 3*4";
			exit(0);
		}
		pre_transformation=CGAL::Aff_transformation_3<K>(
			args["matrix"][0*4+0].asDouble(),args["matrix"][0*4+1].asDouble(),args["matrix"][0*4+2].asDouble(),args["matrix"][0*4+3].asDouble(),
			args["matrix"][1*4+0].asDouble(),args["matrix"][1*4+1].asDouble(),args["matrix"][1*4+2].asDouble(),args["matrix"][1*4+3].asDouble(),
			args["matrix"][2*4+0].asDouble(),args["matrix"][2*4+1].asDouble(),args["matrix"][2*4+2].asDouble(),args["matrix"][2*4+3].asDouble()
			);
	}

	KDTree kdtree_gtpoints;
	KDTree kdtree_reconstructed_points;
	Triangle_Tree* bvhtree_reconstructed_mesh=nullptr;
	Triangle_Tree* bvhtree_gtmesh=nullptr;

	PointSet3 gt_point_set(false);
	std::vector<Point3> gt_vertices;
	std::vector<Vector3> gt_normals;
	std::vector<CGAL::Triangle_3<K>> gt_faces;
	std::vector<std::array<int, 3>> gt_face_indexes;
	LOG(INFO) << "Read gt model";
	if(args.isMember("gt_points") && args.isMember("gt_model"))
	{
		LOG(INFO) << "Define both gt points and model, please specific one source";
		return 0;
	}
	else if(!args.isMember("gt_points") && !args.isMember("gt_model"))
	{
		LOG(INFO) << "Do not define gt points and model at the same time, please specific one source";
		return 0;
	}
	else if(args.isMember("gt_points"))
	{
		if (!CGAL::IO::read_point_set(args["gt_points"].asString(), gt_point_set))
		{
			LOG(ERROR) << "Cannot read " << args["gt_points"].asString();
			exit(0);
		}
		LOG(INFO) << "Read " << gt_point_set.size() << " gt points done";
		kdtree_gtpoints.insert(gt_point_set.points().begin(), gt_point_set.points().end());
		kdtree_gtpoints.build();
	}
	else
	{
		modeltools::read_model(args["gt_model"].asString(), gt_vertices, gt_normals, gt_face_indexes, gt_faces);
		bvhtree_gtmesh = new Triangle_Tree(gt_faces.begin(), gt_faces.end());
		bvhtree_gtmesh->build();
		bvhtree_gtmesh->accelerate_distance_queries(gt_vertices.begin(), gt_vertices.end());

		gt_point_set = modeltools::sample_points_according_density(gt_faces,1000);
		LOG(INFO) << "Read GT model done. It has " << gt_vertices.size() << " points and " << gt_faces.size() << " faces";
		LOG(INFO) << "Sampling points on the mesh according to density 1000/m2, result in " << gt_point_set.size() << " points";
		kdtree_gtpoints.insert(gt_point_set.points().begin(), gt_point_set.points().end());
		kdtree_gtpoints.build();
	}

	LOG(INFO) << "Read reconstructed model";
	PointSet3 reconstructed_point_set(true);
	std::vector<Point3> reconstructed_vertices;
	std::vector<Vector3> reconstructed_normals;
	std::vector<CGAL::Triangle_3<K>> reconstructed_faces;
	std::vector<std::array<int,3>> reconstructed_face_indexes;

	if(args.isMember("recon_points") && args.isMember("recon_model"))
	{
		LOG(INFO) << "Define both reconstructed points and model, please specific one source";
		return 0;
	}
	else if(!args.isMember("recon_points") && !args.isMember("recon_model"))
	{
		LOG(INFO) << "Do not define reconstructed points and model at the same time, please specific one source";
		return 0;
	}
	else if(args.isMember("recon_points"))
	{
		if (!CGAL::IO::read_point_set(args["recon_points"].asString(), reconstructed_point_set))
		{
			LOG(ERROR) << "Cannot read " << args["recon_points"].asString();
			exit(0);
		}
		for (auto& item : reconstructed_point_set.points())
			item = pre_transformation(item);
		for (auto& item : reconstructed_point_set.normals())
			item = pre_transformation(item);
		LOG(INFO) << "Read " << reconstructed_point_set.size() << " recon points done";
		kdtree_reconstructed_points.insert(reconstructed_point_set.points().begin(), reconstructed_point_set.points().end());
		kdtree_reconstructed_points.build();
	}
	else
	{
		modeltools::read_model(args["recon_model"].asString(),reconstructed_vertices,reconstructed_normals, reconstructed_face_indexes,reconstructed_faces);
#pragma omp parallel for
		for(int i=0;i<reconstructed_vertices.size();++i)
			reconstructed_vertices[i]=pre_transformation(reconstructed_vertices[i]);
#pragma omp parallel for
		for (int i = 0;i < reconstructed_normals.size();++i)
			reconstructed_normals[i] = pre_transformation(reconstructed_normals[i]);
#pragma omp parallel for
		for(int i=0;i<reconstructed_faces.size();++i)
			reconstructed_faces[i]=reconstructed_faces[i].transform(pre_transformation);

		reconstructed_point_set = modeltools::sample_points_according_density(reconstructed_faces, 1000);
		//for (int i = 0;i < reconstructed_vertices.size();++i)
		//	reconstructed_point_set.insert(reconstructed_vertices[i], reconstructed_normals[i]);
		//CGAL::IO::write_PLY("test.ply",reconstructed_faces);
		//reconstructed_point_set = sample_points(reconstructed_mesh,1000000);
		LOG(INFO) << "Read Recon model done. It has " << reconstructed_vertices.size() << " points and " << reconstructed_faces.size() << " faces";
		LOG(INFO) << "Sampling points on the mesh according to density 1000/m2, result in " << reconstructed_point_set.size() << " points";
		LOG(INFO) << "Start to build tree";
		bvhtree_reconstructed_mesh = new Triangle_Tree(reconstructed_faces.begin(),reconstructed_faces.end());
		bvhtree_reconstructed_mesh->build();
		bvhtree_reconstructed_mesh->accelerate_distance_queries(reconstructed_vertices.begin(), reconstructed_vertices.end());
		kdtree_reconstructed_points.insert(reconstructed_point_set.points().begin(), reconstructed_point_set.points().end());
		kdtree_reconstructed_points.build();
	}
	CGAL::IO::write_point_set("sample_reconstructed_model.ply",reconstructed_point_set);

	LOG(INFO) << "Start local ICP";
	int num_try = args["local_icp"].asInt();
	while(num_try >0)
	{
		LOG(INFO) << args["local_icp"].asInt() - num_try <<  " times local ICP";

		double distance=9999.;

		PointSet3 aligned_pointset = modeltools::sample_points_according_density(reconstructed_faces, 50);
		PointSet3 ref_pointset;
		if (gt_faces.size() != 0)
			ref_pointset = modeltools::sample_points_according_density(gt_faces, 50);
		else
			ref_pointset = gt_point_set;
		CGAL::IO::write_point_set("aligned_pointset.ply", aligned_pointset);
		CGAL::IO::write_point_set("ref_pointset.ply", ref_pointset);
		Eigen::Matrix4d transformation = estimate_transform_plus(
			aligned_pointset,
			ref_pointset,
			100,distance,
			bvhtree_reconstructed_mesh,bvhtree_gtmesh);

		CGAL::Aff_transformation_3<K> cgal_transformation(
			transformation(0,0),transformation(0,1),transformation(0,2),transformation(0,3),
			transformation(1,0),transformation(1,1),transformation(1,2),transformation(1,3),
			transformation(2,0),transformation(2,1),transformation(2,2),transformation(2,3)
			);
		pre_transformation = cgal_transformation * pre_transformation;
		num_try--;
#pragma omp parallel for
		for (int i = 0;i < reconstructed_point_set.size();++i)
		{
			reconstructed_point_set.point(i) = cgal_transformation(reconstructed_point_set.point(i));
			reconstructed_point_set.normal(i) = cgal_transformation(reconstructed_point_set.normal(i));
		}
#pragma omp parallel for
		for(int i=0;i<reconstructed_vertices.size();++i)
			reconstructed_vertices[i]=cgal_transformation(reconstructed_vertices[i]);
#pragma omp parallel for
		for(int i=0;i<reconstructed_faces.size();++i)
			reconstructed_faces[i]=reconstructed_faces[i].transform(cgal_transformation);

		if(bvhtree_reconstructed_mesh!=nullptr)
		{
			bvhtree_reconstructed_mesh->clear();
			bvhtree_reconstructed_mesh->~AABB_tree();
			bvhtree_reconstructed_mesh=new Triangle_Tree(reconstructed_faces.begin(),reconstructed_faces.end());
			bvhtree_reconstructed_mesh->build();
			bvhtree_reconstructed_mesh->accelerate_distance_queries(reconstructed_vertices.begin(), reconstructed_vertices.end());
			kdtree_reconstructed_points.clear();
			kdtree_reconstructed_points.insert(reconstructed_point_set.points().begin(), reconstructed_point_set.points().end());
			kdtree_reconstructed_points.build();
		}
		else
		{
			kdtree_reconstructed_points.clear();
			kdtree_reconstructed_points.insert(reconstructed_point_set.points().begin(), reconstructed_point_set.points().end());
			kdtree_reconstructed_points.build();
		}
	}
	CGAL::IO::write_point_set("fine_aligned_reconstructed_model.ply",reconstructed_point_set);
	LOG(INFO)<<"Here is the local icp transformation";
	std::cout<<pre_transformation<<std::endl;

	const bool is_filter_z = false;
	const float filter_z = 1.2f;
	if(is_filter_z)
	{
		const int original_size = reconstructed_point_set.size();
		for (int i = reconstructed_point_set.size() - 1; i >= 0; --i)
			if (reconstructed_point_set.point(i).z() < filter_z)
				reconstructed_point_set.remove(i);
		reconstructed_point_set.collect_garbage();
		LOG(INFO) << boost::format("Filter out %d points whose z value is less than %f; Remain %d points") % (original_size - reconstructed_point_set.size()) % filter_z % reconstructed_point_set.size();
	}


	LOG(INFO) << "Start to calculate accuracy";
	std::vector<double> accuracy(reconstructed_point_set.size());
#pragma omp parallel for
	for(int i_point=0;i_point<reconstructed_point_set.size();++i_point)
	{
		const auto& point2 = reconstructed_point_set.point(i_point);
		if(args.isMember("gt_points"))
		{
			Neighbor_search search(kdtree_gtpoints, point2, 1);
			for (Neighbor_search::iterator it = search.begin(); it != search.end(); ++it)
				accuracy[i_point]=std::sqrt(it->second);
		}
		else
		{
			accuracy[i_point]=std::sqrt(CGAL::squared_distance(bvhtree_gtmesh->closest_point(point2),point2));
		}
	}

	LOG(INFO) << "Start to calculate completeness";
	if (is_filter_z)
	{
		const int original_size = gt_point_set.size();
		for (int i = gt_point_set.size() - 1; i >= 0; --i)
			if (gt_point_set.point(i).z() < filter_z)
				gt_point_set.remove(i);
		gt_point_set.collect_garbage();
		LOG(INFO) << boost::format("Filter out %d points whose z value is less than %f; Remain %d points") % (original_size - gt_point_set.size()) % filter_z % gt_point_set.size();
	}

	std::vector<double> completeness(gt_point_set.size());
#pragma omp parallel for
	for(int i_point=0;i_point<gt_point_set.size();++i_point)
	{
		const auto& point1 = gt_point_set.point(i_point);
		if(args.isMember("recon_points"))
		{
			Neighbor_search search(kdtree_reconstructed_points, point1, 1);
			for (Neighbor_search::iterator it = search.begin(); it != search.end(); ++it)
				completeness[i_point]=std::sqrt(it->second);
		}
		else
		{
			completeness[i_point]=std::sqrt(bvhtree_reconstructed_mesh->squared_distance(point1));
		}
	}

	auto gt_error_map = gt_point_set.add_property_map<double>("error").first;
	auto recon_error_map = reconstructed_point_set.add_property_map<double>("error").first;

	for(const auto& i_point:gt_point_set)
		gt_error_map[i_point] = completeness[i_point];
	for(const auto& i_point:reconstructed_point_set)
		recon_error_map[i_point] = accuracy[i_point];

	std::ofstream a("recon_sample_points_with_error.ply",std::ios::binary);
	a.precision(10);
	CGAL::set_binary_mode(a);
	CGAL::IO::write_PLY_with_properties(a,reconstructed_point_set,
		CGAL::IO::make_ply_point_writer(reconstructed_point_set.point_map()),
		std::make_pair(recon_error_map,CGAL::PLY_property<double>("error"))
	);
	
	a.close();
	std::ofstream b("gt_sample_points_with_error.ply",std::ios::binary);
	b.precision(10);
	CGAL::set_binary_mode(b);
	CGAL::IO::write_PLY_with_properties(b,gt_point_set,
		CGAL::IO::make_ply_point_writer(gt_point_set.point_map()),
		std::make_pair(gt_error_map,CGAL::PLY_property<double>("error"))
	);
	b.close();

	LOG(INFO) << "Start to sort";
	std::vector<double> sorted_accuracy(accuracy);
	std::sort(sorted_accuracy.begin(), sorted_accuracy.end());
	std::sort(completeness.begin(), completeness.end());
	
	std::vector<double> accuracy_threshold{
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 50)] ,
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 60)] ,
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 70)] ,
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 80)] ,
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 90)] ,
		sorted_accuracy[(int)(sorted_accuracy.size() / 100 * 95)]
	};

	std::vector<PointSet3> point_set_accuracy_outlier(accuracy_threshold.size());
	
	LOG(INFO) << "Start to extract";
	for (int i_point = 0;i_point < reconstructed_point_set.size();++i_point)
	{
		for (int i_threshold = 0; i_threshold < accuracy_threshold.size(); ++i_threshold)
		{
			if (accuracy[i_point] > accuracy_threshold[i_threshold])
				point_set_accuracy_outlier[i_threshold].insert(reconstructed_point_set.point(i_point));
		}
	}

	for (int i_threshold = 0; i_threshold < accuracy_threshold.size(); ++i_threshold)
	{
		std::ofstream f("accuracy_outlier_" + std::to_string(accuracy_threshold[i_threshold]) + ".ply",
			std::ios::out | std::ios::binary);
		CGAL::set_binary_mode(f);
		CGAL::write_ply_point_set(f, point_set_accuracy_outlier[i_threshold]);
	}
	LOG(INFO) << "50% Accuracy: " << accuracy_threshold[0];
	LOG(INFO) << "60% Accuracy: " << accuracy_threshold[1];
	LOG(INFO) << "70% Accuracy: " << accuracy_threshold[2];
	LOG(INFO) << "80% Accuracy: " << accuracy_threshold[3];
	LOG(INFO) << "90% Accuracy: " << accuracy_threshold[4];
	LOG(INFO) << "95% Accuracy: " << accuracy_threshold[5];

	LOG(INFO) << "50% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.5),0.f)/(int)(sorted_accuracy.size()*0.5);
	LOG(INFO) << "60% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.6),0.f)/(int)(sorted_accuracy.size()*0.6);
	LOG(INFO) << "70% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.7),0.f)/(int)(sorted_accuracy.size()*0.7);
	LOG(INFO) << "80% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.8),0.f)/(int)(sorted_accuracy.size()*0.8);
	LOG(INFO) << "90% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.9),0.f)/(int)(sorted_accuracy.size()*0.9);
	LOG(INFO) << "95% Average: " << std::accumulate(sorted_accuracy.begin(), sorted_accuracy.begin() + (int)(sorted_accuracy.size()*0.95),0.f)/(int)(sorted_accuracy.size()*0.95);

	LOG(INFO) << "Start to compute completeness";
	std::vector<double> completeness_threshold{ 0.01,0.02, 0.03,0.05, 0.1,0.2,0.5,1 };
	std::vector<double> completeness_range(completeness_threshold.size(), 0);
	for(int i_point=0;i_point< completeness.size();i_point++)
	{
		for (int i_threshold = 0; i_threshold < completeness_threshold.size(); ++i_threshold)
		{
			if (completeness_range[i_threshold] == 0 && completeness[i_point] > completeness_threshold[i_threshold])
				completeness_range[i_threshold] = (double)i_point / completeness.size() * 100;
		}
	}

	for (int i_threshold = 0; i_threshold < completeness_threshold.size(); ++i_threshold)
		LOG(INFO)<< completeness_range[i_threshold] << "% of points has error lower than " << completeness_threshold[i_threshold];
}
