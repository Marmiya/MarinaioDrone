//#include "ppc.h"
//
//std::vector<Viewpoint> pathplanning(
//	PointSet3& points, const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
//	const int maxIterTimes, const int initViewNum, const std::string& logPath,
//	const double viewDis, const double maxAngle, const double maxDis
//)
//{
//	std::cout << "Start first DroneScan.\n\n";
//	auto traj = droneScan(
//		points, mesh, intrinsicMatrix, 
//		maxIterTimes, initViewNum, logPath,
//		viewDis, maxAngle, maxDis
//	);
//	CGAL::IO::write_point_set(logPath + "finalpts.ply", points);
//	write_normal_path(traj, logPath + "initialans.txt");
//	std::cout << "Finish first DroneScan.\n\n";
//
//	pathplanningAdj(points, traj, logPath, mesh,
//		intrinsicMatrix, viewDis, maxAngle, maxDis
//	);
//
//	return traj;
//}
//
//
//void pathplanningAdj(
//	const PointSet3& points, std::vector<Viewpoint>& traj, const std::string& logPath,
//	const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
//	const double viewDis, const double maxAngle, const double maxDis
//)
//{
//	modeltools::Height_map height_map;
//	// Height map generation
//	{
//		double area = CGAL::Polygon_mesh_processing::area(mesh);
//		int num_sample = area / std::sqrt(3);
//		PointSet3 dense_points_used_for_heightmap_generation;
//		CGAL::Random_points_in_triangle_mesh_3<SurfaceMesh> dense_point_generator(mesh);
//		std::copy_n(dense_point_generator, num_sample, dense_points_used_for_heightmap_generation.point_back_inserter());
//		height_map = modeltools::Height_map(dense_points_used_for_heightmap_generation, 3, 2, 6, 0);
//		height_map.save_height_map_mesh(logPath);
//		CGAL::IO::write_point_set(logPath + "dense_sample_points.ply", dense_points_used_for_heightmap_generation);
//		LOG(INFO) << boost::format("Generate height map done using %d sample points. Size: %d/%d") % num_sample % height_map.m_map.rows % height_map.m_map.cols;
//	}
//
//	int trajSize = traj.size();
//	int lastSize = trajSize + 11;
//
//	int itrcnt = 0;
//	while (lastSize - trajSize > 10)
//	{
//		itrcnt++;
//		traj = adjustTraj(
//			traj, height_map,
//			points, mesh,
//			intrinsicMatrix, maxDis, 42
//		);
//
//		traj = droneScanAdj(points, traj, mesh, intrinsicMatrix, viewDis, maxAngle, maxDis);
//
//		lastSize = trajSize;
//		trajSize = traj.size();
//		LOG(INFO) << "last: " << lastSize << "\tnow: " << trajSize;
//		std::cout << "finish one iteration" << std::endl;
//
//		write_normal_path(traj, logPath + std::to_string(itrcnt) + std::string(".txt"));
//	}
//	
//}
//
//std::vector<Viewpoint> viewAddtion(
//	const PointSet3& pts, const std::vector<Viewpoint>& traj, const std::string& logPath,
//	const SurfaceMesh& mesh, const modeltools::Height_map& heightMap,
//	const Eigen::Matrix3d& intrinsicMatrix,	const double viewDis, const double maxAngle, const double maxDis
//)
//{
//	const int pSize = static_cast<int>(pts.size());
//	std::vector<Eigen::Vector3d> ptsp(pSize), ptsn(pSize);
//	for (int i = 0; i < pSize; i++)
//	{
//		ptsp.at(i) = cgaltools::cgal_point_2_eigen(pts.point(i));
//		ptsn.at(i) = cgaltools::cgal_vector_2_eigen(pts.normal(i));
//	}
//	std::vector<Viewpoint> ans = traj;
//	const auto embree_scene = intersectiontools::generate_embree_scene(mesh);
//	auto visibilityindex = compute_visibilityIndex(ans, mesh, pts, intrinsicMatrix, maxDis, embree_scene);
//	auto curREC = totalRECv(ans, ptsp, ptsn, visibilityindex);
//	for (int i = 0; i < pSize; ++i)
//	{
//		if (curREC.at(i) < 20)
//		{
//			
//		}
//	}
//}