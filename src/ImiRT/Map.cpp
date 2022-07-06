#include "Map.h"

GTMap::GTMap(const string& modelPath, 
	const double& ID, const double& UID, const double& filterHeight)
{
	surfaceMesh = modeltools::read_model(modelPath);
	SurfaceMesh importantModel = surfaceMesh;
	SurfaceMesh unimportantModel = surfaceMesh;

	for (auto face : importantModel.faces())
	{
		bool remove = true;
		for (auto vertex : importantModel.vertices_around_face(importantModel.halfedge(face)))
		{
			if (importantModel.point(vertex).z() > filterHeight)
				remove = false;
		}
		if (remove)
			CGAL::remove_face(face, importantModel);
	}
	for (auto face : unimportantModel.faces())
	{
		bool remove = true;
		for (auto vertex : unimportantModel.vertices_around_face(unimportantModel.halfedge(face)))
		{
			if (unimportantModel.point(vertex).z() > filterHeight)
				remove = false;
		}
		if (!remove)
			CGAL::remove_face(face, unimportantModel);
	}

	CGAL::collect_garbage(importantModel);
	CGAL::collect_garbage(unimportantModel);

	double important_area = CGAL::Polygon_mesh_processing::area(importantModel);
	double unimportant_area = CGAL::Polygon_mesh_processing::area(unimportantModel);
	double total_area = important_area + unimportant_area;

	int num_important_sample = important_area * ID;
	int num_unimportant_sample = unimportant_area * UID;
	PointSet3 important_sample_points = modeltools::sample_points(importantModel, num_important_sample);
	PointSet3 unimportant_sample_points = modeltools::sample_points(unimportantModel, num_unimportant_sample);

	PointSet3 whole_points(important_sample_points);
	for (auto point : unimportant_sample_points)
		whole_points.insert(unimportant_sample_points.point(point), unimportant_sample_points.normal(point));

	samplePts = std::move(whole_points);
}

GTMap::GTMap(const string& modelPath, const string& ptsPath)
{
	surfaceMesh = modeltools::read_model(modelPath);
	CGAL::IO::read_point_set(ptsPath, samplePts);
}

void GTMap::initialization(const double& CCL)
{
	range = { surfaceMesh.point(SMVI(0)).x(),surfaceMesh.point(SMVI(0)).x(),
		surfaceMesh.point(SMVI(0)).y(),surfaceMesh.point(SMVI(0)).y() };
	for (const auto& p : surfaceMesh.points())
	{
		if (p.x() < std::get<0>(range))
		{
			std::get<0>(range) = p.x();
		}
		if( p.x() > std::get<1>(range))
		{
			std::get<1>(range) = p.x();
		}
		if (p.y() < std::get<2>(range))
		{
			std::get<2>(range) = p.y();
		}
		if (p.y() > std::get<3>(range))
		{
			std::get<3>(range) = p.y();
		}
	}
	// X/Y axis grid point amount
	int xpa = static_cast<int>((std::get<1>(range) - std::get<0>(range)) / CCL);
	int ypa = static_cast<int>((std::get<3>(range) - std::get<2>(range)) / CCL);
	xGridPoint.push_back(std::get<0>(range));
	yGridPoint.push_back(std::get<2>(range));
	for (int i = 1; i <= xpa; i++)
	{
		xGridPoint.push_back(i * CCL);
	}
	for (int i = 1; i <= ypa; i++)
	{
		yGridPoint.push_back(i * CCL);
	}
	xGridPoint.push_back(std::get<1>(range));
	yGridPoint.push_back(std::get<3>(range));

	rtcScene = intersectiontools::generate_embree_scene(surfaceMesh);
}