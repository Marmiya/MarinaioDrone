#pragma once

//#define TINYOBJLOADER_IMPLEMENTATION
// TINYOBJLOADER_USE_MAPBOX_EARCUT: Enable better triangulation. Requires C++11

#include <tuple>
#include <iostream>
#include <corecrt_math.h>

#include <boost/core/swap.hpp>

#include <CGAL/AABB_tree.h>
#include <CGAL/approximated_offset_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Plane_3.h>
#include <CGAL/Point_set_2.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Random.h>
#include <CGAL/Surface_mesh/Surface_mesh.h>
#include <opencv2/opencv.hpp>

#include <Eigen/src/Geometry/AlignedBox.h>
#include <tiny_obj_loader.h>

// For intersection_tools.h
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_traits.h>
#include <CGAL/AABB_triangle_primitive.h>
#include <CGAL/AABB_face_graph_triangle_primitive.h>
#include <CGAL/Point_set_3.h>


using K = CGAL::Exact_predicates_inexact_constructions_kernel;

using IsoCuboid3 = K::Iso_cuboid_3;
using Plane3 = K::Plane_3;
using Point2 = K::Point_2;
using Polygon2 = CGAL::Polygon_2<K>;
using PointSet2 = CGAL::Point_set_2<Point2>;
using Point3 = K::Point_3;
using PointSet3 = CGAL::Point_set_3<Point3>;
using Polyhedron3 = CGAL::Polyhedron_3<K>;
using Ray3 = K::Ray_3;
using Segment3 = K::Segment_3;
using SurfaceMesh = CGAL::Surface_mesh<K::Point_3>;
using Tetrahedron3 = K::Tetrahedron_3;
using Triangle3 = K::Triangle_3;
using Vector3 = CGAL::Vector_3<K>;
// For intersection_tools.h
using Primitive = CGAL::AABB_face_graph_triangle_primitive<SurfaceMesh, CGAL::Default, CGAL::Tag_false>;
using AABB_Traits = CGAL::AABB_traits<K, Primitive>;
using Tree = CGAL::AABB_tree<AABB_Traits>;
using Primitive_id = Tree::Primitive_id;
using KDTreeTraits = CGAL::Search_traits_3<K>;
using Neighbor_search = CGAL::Orthogonal_k_neighbor_search<KDTreeTraits>;
using KDTree = Neighbor_search::Tree;

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
		float angle;
		RotatedBox() {};
		RotatedBox(const Eigen::AlignedBox3d& v_box) :box(v_box), angle(0.f)
		{
			cv_box = cv::RotatedRect(cv::Point2f(v_box.center().x(), v_box.center().y()), cv::Size2f(v_box.sizes().x(), v_box.sizes().y()), 0.f);
		}
		RotatedBox(const Eigen::AlignedBox3d& v_box, float v_angle_in_degree) :box(v_box), angle(v_angle_in_degree / 180.f * 3.1415926)
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
