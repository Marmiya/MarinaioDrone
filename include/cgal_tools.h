#pragma once

//#define TINYOBJLOADER_IMPLEMENTATION
// TINYOBJLOADER_USE_MAPBOX_EARCUT: Enable better triangulation. Requires C++11

#include <tuple>
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <corecrt_math_defines.h>

#include <boost/core/swap.hpp>

#include <CGAL/AABB_tree.h>
#include <CGAL/approximated_offset_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/cluster_point_set.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/grid_simplify_point_set.h>
#include <CGAL/Plane_3.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/Point_set_2.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_slicer.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/poisson_surface_reconstruction.h>
#include <CGAL/property_map.h>
#include <CGAL/Random.h>
#include <CGAL/random_selection.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>

#include <opencv2/opencv.hpp>

#include <Eigen/src/Geometry/AlignedBox.h>
#include <tiny_obj_loader.h>

#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>

#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing_on_point_set.h>
#include <CGAL/Polygonal_surface_reconstruction.h>
#include <CGAL/SCIP_mixed_integer_program_traits.h>

using K = CGAL::Exact_predicates_inexact_constructions_kernel;

using IsoCuboid3 = K::Iso_cuboid_3;
using Plane3 = K::Plane_3;
using Point2 = K::Point_2;
using Polygon2 = CGAL::Polygon_2<K>;
using PointSet2 = CGAL::Point_set_2<Point2>;
using Point3 = K::Point_3;
using Polyhedron3 = CGAL::Polyhedron_3<K>;
using Ray3 = K::Ray_3;
using Segment2 = K::Segment_2;
using Segment3 = K::Segment_3;
using Direction2 = K::Direction_2;
using SurfaceMesh = CGAL::Surface_mesh<K::Point_3>;
using Tetrahedron3 = K::Tetrahedron_3;
using Triangle3 = K::Triangle_3;
using Vector2 = K::Vector_2;
using Vector3 = K::Vector_3;
using PointSet3 = CGAL::Point_set_3<Point3, Vector3>;
using Primitive = CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh, CGAL::Default, CGAL::Tag_false>;
using AABB_Traits = CGAL::AABB_traits<K, Primitive>;
using Tree = CGAL::AABB_tree<AABB_Traits>;
using Primitive_id = Tree::Primitive_id;
using KDTreeTraits = CGAL::Search_traits_3<K>;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<KDTreeTraits>;
using KDTree = Neighbor_search::Tree;
using Polyline_type = std::vector<Point3>;
using Polylines = std::list<Polyline_type>;

using PNI = boost::tuple<Point3, Vector3, int>;
using Point_vector = std::vector<PNI>;
using Point_map = CGAL::Nth_of_tuple_property_map<0, PNI>;
using Normal_map = CGAL::Nth_of_tuple_property_map<1, PNI>;
using Plane_index_map = CGAL::Nth_of_tuple_property_map<2, PNI>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<K, Point_vector, Point_map, Normal_map>;
using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using Plane = CGAL::Shape_detection::Plane<Traits>;
using Point_to_shape_index_map = CGAL::Shape_detection::Point_to_shape_index_map<Traits>;
using Polygonal_surface_reconstruction = CGAL::Polygonal_surface_reconstruction<K>;
using MIP_Solver = CGAL::SCIP_mixed_integer_program_traits<double>;

using FT = K::FT;
using Neighbor_query = CGAL::Shape_detection::Point_set::Sphere_neighbor_query<K, Point_vector, Point_map>;
using Region_type = CGAL::Shape_detection::Point_set::Least_squares_plane_fit_region<K, Point_vector, Point_map, Normal_map>;
using Region_growing = CGAL::Shape_detection::Region_growing<Point_vector, Neighbor_query, Region_type>;

class Index_map {
public:
	using key_type = std::size_t;
	using value_type = int;
	using reference = value_type;
	using category = boost::readable_property_map_tag;
	Index_map() { }
	template<typename PointRange>
	Index_map(const PointRange& points,
		const std::vector< std::vector<std::size_t> >& regions)
		: m_indices(new std::vector<int>(points.size(), -1))
	{
		for (std::size_t i = 0; i < regions.size(); ++i)
			for (const std::size_t idx : regions[i])
				(*m_indices)[idx] = static_cast<int>(i);
	}
	inline friend value_type get(const Index_map& index_map,
		const key_type key)
	{
		const auto& indices = *(index_map.m_indices);
		return indices[key];
	}
private:
	std::shared_ptr< std::vector<int> > m_indices;
};



namespace cgaltools {

	// Link node of segments
	class SegmentLN {
	private:
		int visited = 0;
		Segment3 seg;
		std::shared_ptr<SegmentLN> next = nullptr;

	public:
		SegmentLN() = default;
		SegmentLN(Segment3 seg) :seg(seg) {};
		SegmentLN(Segment3 seg, std::shared_ptr<SegmentLN> nxt) :
			seg(seg), next(nxt) {};

		void setNext(std::shared_ptr<SegmentLN> nxt) {
			next = nxt;
		}
		std::shared_ptr<SegmentLN> getNext() {
			return next;
		}
		std::shared_ptr<SegmentLN> getLast() {
			auto last = next;
			while (last->getNext()->getSeg() != seg) {
				last = last->getNext();
			}
			return last;
		}
		Segment3& getSeg() {
			return seg;
		}
		void setSeg(Segment3 s) {
			seg = s;
		}
		int ifvisited() {
			return visited;
		}
		void visit() {
			visited += 1;
		}
	};


	class Cuboid {

	private:
		// The hexahedron constructed by this tetrahedron
		Tetrahedron3 base;
		// All the vertices on the hexahedron, totally 8.
		// The vertices are sorted as below:
		// First point is the nearest point to the origin on the underside.
		// Then three points are the other points on the underside in counterclockwise order.
		// The next point is the correspondance of the last point on the underside.
		// The last three points just like above.
		std::vector<Point3> vertices;
		// All the segments, the order along vertices, totally 12.
		std::vector<Segment3> segments;

		std::shared_ptr<SegmentLN> bottomSegs;

	public:

		Cuboid() = default;
		Cuboid(std::set<Point3>);

		std::vector<Point3> getVer() {
			return vertices;
		}
		std::vector<Segment3> getSeg() {
			return segments;
		}
		std::shared_ptr<SegmentLN> getBottomSegs() {
			return bottomSegs;
		}


	};

	struct RotatedBox
	{
		cv::RotatedRect cv_box;
		Eigen::AlignedBox3d box;
		double angle;
		RotatedBox() {};
		RotatedBox(const Eigen::AlignedBox3d& v_box) :box(v_box), angle(0.)
		{
			cv_box = cv::RotatedRect(cv::Point2f(v_box.center().x(), v_box.center().y()), cv::Size2f(v_box.sizes().x(), v_box.sizes().y()), 0.f);
		}
		RotatedBox(const Eigen::AlignedBox3d& v_box, float v_angle_in_degree) :box(v_box), angle(v_angle_in_degree / 180. * 3.1415926)
		{
			cv_box = cv::RotatedRect(cv::Point2f(v_box.center().x(), v_box.center().y()), cv::Size2f(v_box.sizes().x(), v_box.sizes().y()), v_angle_in_degree);
		}

		bool inside_2d(const Eigen::Vector3d& v_point) const
		{
			Eigen::Vector2d point(v_point.x(), v_point.y());
			float s = std::sin(-angle);
			float c = std::cos(-angle);

			// set origin to rect center
			Eigen::Vector2d newPoint = point - Eigen::Vector2d(box.center().x(), box.center().y());
			// rotate
			newPoint = Eigen::Vector2d(newPoint.x() * c - newPoint.y() * s, newPoint.x() * s + newPoint.y() * c);
			// put origin back
			newPoint = newPoint + Eigen::Vector2d(box.center().x(), box.center().y());
			if (newPoint.x() >= box.min().x() && newPoint.x() <= box.max().x() && newPoint.y() >= box.min().y() && newPoint.y() <= box.max().y())
				return true;
			else
				return false;
		}

	};

	// The way these functions judge if two cuboids contact is by checking their undersides.
	// Hence some error may exsit. 
	double squaredDistanceCuboid(Cuboid&, Cuboid&);

	// Point to viewer
	void pointVisualization(
		std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>>&,
		Point3, double);

	void pointVisualizationPyramid(
		std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>>&,
		Point3, double);

	std::tuple<std::shared_ptr<SegmentLN>, int> makeProfile(std::vector<SegmentLN>);

	std::tuple<std::shared_ptr<SegmentLN>, int> makeProfileInterval(std::vector<SegmentLN>);
	
	std::vector < std::pair<std::shared_ptr<SegmentLN>, Plane3>>
		getsides(std::vector<Cuboid>);

	// @brief: 
	// @notice: Currently only transfer vertices to the cgal Surface mesh
	// @param: `attrib_t, shape_t, material_t`
	// @ret: Surface_mesh
	SurfaceMesh convert_obj_from_tinyobjloader_to_surface_mesh(
		const std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>> v_obj_in);

	Eigen::AlignedBox3d get_bounding_box(const PointSet3& v_point_set);
	RotatedBox get_bounding_box_rotated(const PointSet3& v_point_set);

	Eigen::Vector3d cgal_point_2_eigen(const Point3& p);

	Eigen::Vector3d cgal_vector_2_eigen(const Vector3& p);

	Point3 eigen_2_cgal_point(const Eigen::Vector3d& p);

	Vector3 eigen_2_cgal_vector(const Eigen::Vector3d& p);

}
