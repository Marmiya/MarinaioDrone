#ifndef MODEL_TOOLS_H
#define MODEL_TOOLS_H

#include <opencv2/opencv.hpp>
#include "cgal_tools.h"
#include "common_util.h"

#include <queue>
#include <fstream>
#include <random>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <Eigen/Core>


namespace modeltools{

    // Load obj file by tinyobj interface.
    std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>>
	load_obj (
        const std::string& v_mesh_name, bool v_log = true, const std::string& v_mtl_dir = "./");

    std::tuple<std::shared_ptr<cgaltools::SegmentLN>, double, double>
        safeExpansion(
            std::shared_ptr<cgaltools::SegmentLN>, double);

    Polygon2 safeExpansionAny(
        std::vector<cgaltools::SegmentLN>, double);
    
    //It's wrong now. Just a interpolation process.
    PointSet3 CCPP(Point3, Point3);

    struct Proxy
    {
        float height;
        CGAL::Polygon_2<K> polygon;
    };

    SurfaceMesh read_model(const fs::path& v_path);
    bool read_model(const fs::path& v_path, Polyhedron3& v_mesh);


    /*
    Some useful function
    */

    // @brief: Get polygon represented by footprint
    // @notice: Footprint format:
    // height
    // "1th vertex x" "1th vertex y" "2th vertex x" "2th vertex y" ...
    // @param: File path
    // @ret: Proxy
    Proxy load_footprint(const std::string& v_footprint_path);

    // @brief: Store mesh into obj file
    // @notice: Every shape in the vector will have a group name
    // @param: File path; attrib_t; shape_t; material_t;
    // @ret:
    bool write_obj(const std::string& filename, const tinyobj::attrib_t& attributes, const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials);

    // @brief: Meshlab save obj file with an overiding material name. e.g. material_0, material_1... This function change the original mtl file into the meshlab style material name
    // @notice: 
    // @param: mtl file path
    // @ret:
    void fix_mtl_from_unreal(const std::string& filename);

    // @brief: Clean duplicated face and vertex
    // @notice: Currently implementation is only support shape with normals and texcoods!
    // @param: attrib_t, shape_t
    // @ret:
    void clean_vertex(tinyobj::attrib_t& attrib, tinyobj::shape_t& shape);

    // @brief: Clean unused materials
    // @notice: 
    // @param: shape_t, vector<material_t>
    // @ret:
    void clean_materials(tinyobj::shape_t& shape, std::vector<tinyobj::material_t>& materials);


    /*
    Get split mesh with a big whole mesh
    */
    void merge_obj(const std::string& v_file,
        const std::vector<tinyobj::attrib_t>& v_attribs, const std::vector<std::vector<tinyobj::shape_t>>& saved_shapes,
        const std::vector < std::vector<tinyobj::_material_t>>& materials,
        const int start_id = 0);

    // @brief: Split the whole obj into small object and store them separatly. 
    //         Each Object is store separatly and normalize near origin. The transformation is also stored in the txt
    //         We also get a whole obj with "g" attribute to indicate each component. This obj can be imported into unreal with separate actor
    // @notice: Be careful about the material file, you may adjust them manually
    // @param: 
    //          Directory that contains the obj file. The splited obj will also be stored in this directory
    //          OBJ file name
    //          Resolution indicates the resolution of the height map. (How far will the two building is considered to be one component)
    // @ret:
    void split_obj(const std::string& file_dir, const std::string& file_name, const float resolution, const float v_filter_height = -99999, const int obj_max_builidng_num = -1, const std::string& output_dir = "./", const int split_axis = 2);

    std::vector<tinyobj::shape_t> split_obj_according_to_footprint(const tinyobj::attrib_t& v_attribs, const std::vector<tinyobj::shape_t>& v_shapes, const std::vector<Proxy>& v_proxys, const float v_squared_threshold);

    // @brief: Rename the material and image name
    //         Unreal can not cope with complicate image name
    // @notice: 
    // @param: 
    //          Directory that contains the obj file. 
    //          OBJ file name
    //          Output directory
    // @ret:
    void rename_material(const std::string& file_dir, const std::string& file_name, const std::string& v_output_dir);

    // @brief: Get mesh bounds to determine drone position
    // @notice: 
    // @param: 
    //          Model path 
    //          Safe bounds to determine z range
    // @ret:
    std::vector<float> get_bounds(std::string path, float v_bounds);
    // @brief: Get valid polygons
    // @notice: 
    // @param: 
    //          Point coordinate file of multiple-vertices lines
    // @ret:
    std::vector<Polygon2> get_polygons(std::string file_path);
    // @brief: Sampling points on a surface mesh
    // @notice: 
    // @param: 
    //          SurfaceMesh
    //          num_points
    // @ret:
    PointSet3 sample_points(const SurfaceMesh& v_mesh, const int v_num_points);
    PointSet3 sample_points(const Polyhedron3& v_mesh, const int v_num_points);
    PointSet3 sample_points(const std::vector<Triangle3>& v_mesh, const int v_num_points);

    float point_box_distance_eigen(const Eigen::Vector2d& v_pos, const Eigen::AlignedBox2d& v_box);

    bool inside_box(const Eigen::Vector2d& v_pos, const Eigen::AlignedBox2d& v_box);

    SurfaceMesh get_box_mesh(const std::vector<Eigen::AlignedBox3d>& v_boxes);

    SurfaceMesh get_rotated_box_mesh(const std::vector<cgaltools::RotatedBox>& v_boxes);

    void get_box_mesh_with_colors(const std::vector<Eigen::AlignedBox3d>& v_boxes,
        const std::vector<cv::Vec3b>& v_colors, const std::string& v_name);

    bool liftModel(PointSet3& pts);

    class Height_map {

    public:
        Height_map() {};
        Height_map(const PointSet3& v_point_cloud, const float v_resolution, int v_dilate, float v_default_height = -610610.610610)
            :m_resolution(v_resolution), m_dilate(v_dilate), m_default_height(v_default_height) {
            Eigen::AlignedBox3d bounds = cgaltools::get_bounding_box(v_point_cloud);
            m_start = bounds.min();
            Eigen::Vector3d end(bounds.max());
            Eigen::Vector3d delta = (end - m_start) / m_resolution;
            m_map = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map.setTo(m_default_height);
            m_map_dilated = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map_dilated.setTo(m_default_height);

            for (const auto& id_point : v_point_cloud) {
                const Point3& point = v_point_cloud.point(id_point);
                float cur_height = m_map.at<float>((int)((point.y() - m_start[1]) / m_resolution), (int)((point.x() - m_start[0]) / m_resolution));
                if (cur_height < point.z())
                    m_map.at<float>((int)((point.y() - m_start[1]) / m_resolution), (int)((point.x() - m_start[0]) / m_resolution)) = point.z();
            }
            if (v_dilate != 0)
                cv::dilate(m_map, m_map_dilated,
                    cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(3, 3)),
                    cv::Point(-1, -1),
                    v_dilate);
            else
                m_map_dilated = m_map;
        }

        Height_map(const PointSet3& v_point_cloud, const float v_resolution, int v_dilate, float v_safe_distance, float v_default_height)
            :m_resolution(v_resolution), m_dilate(v_dilate), m_default_height(v_default_height) {
            Eigen::AlignedBox3d bounds = cgaltools::get_bounding_box(v_point_cloud);
            m_start = bounds.min();
            Eigen::Vector3d end(bounds.max());
            Eigen::Vector3d delta = (end - m_start) / m_resolution;
            m_map = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map.setTo(m_default_height);
            m_map_dilated = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map_dilated.setTo(m_default_height);

            for (const auto& id_point : v_point_cloud) {
                const Point3& point = v_point_cloud.point(id_point);
                float cur_height = m_map.at<float>((int)((point.y() - m_start[1]) / m_resolution), (int)((point.x() - m_start[0]) / m_resolution));
                if (cur_height < point.z())
                    m_map.at<float>((int)((point.y() - m_start[1]) / m_resolution), (int)((point.x() - m_start[0]) / m_resolution)) = point.z();
            }
            m_map = m_map + v_safe_distance;
            if (v_dilate != 0)
                cv::dilate(m_map, m_map_dilated,
                    cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(3, 3)),
                    cv::Point(-1, -1),
                    v_dilate);
            else
                m_map_dilated = m_map;
        }

        Height_map(const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max, const float v_resolution, int v_dilate, float v_default_height = -610610.610610)
            :m_resolution(v_resolution), m_start(v_min), m_dilate(v_dilate), m_default_height(v_default_height) {
            Eigen::Vector3d delta = (v_max - m_start) / m_resolution;
            m_map = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map.setTo(m_default_height);
            m_map_dilated = cv::Mat((int)delta[1] + 1, (int)delta[0] + 1, CV_32FC1);
            m_map_dilated.setTo(m_default_height);
        }

        bool is_safe(const Eigen::Vector3d& v_pos) const
        {
            return get_height(v_pos.x(), v_pos.y()) < v_pos.z();
        }

        float get_height(float x, float y) const
        {
            if (has_strict_polygon_boundary && m_strict_boundary.bounded_side(Point2(x, y)) == CGAL::ON_UNBOUNDED_SIDE)
                return std::numeric_limits<float>::max();
            int m_y = (int)((y - m_start[1]) / m_resolution);
            int m_x = (int)((x - m_start[0]) / m_resolution);
            if (!(0 <= m_y && 0 <= m_x && m_y < m_map.rows && m_x < m_map.cols))
                return m_default_height;
            return m_map_dilated.at<float>(m_y, m_x);
        }

        float get_undilated_height(float x, float y) const
        {
            if (has_strict_polygon_boundary && m_strict_boundary.bounded_side(Point2(x, y)) == CGAL::ON_UNBOUNDED_SIDE)
                return std::numeric_limits<float>::max();
            int m_y = (int)((y - m_start[1]) / m_resolution);
            int m_x = (int)((x - m_start[0]) / m_resolution);
            if (!(0 <= m_y && 0 <= m_x && m_y < m_map.rows && m_x < m_map.cols))
                return m_default_height;
            return m_map.at<float>(m_y, m_x);
        }

        bool in_bound(float x, float y) const
        {
            int m_y = (int)((y - m_start[1]) / m_resolution);
            int m_x = (int)((x - m_start[0]) / m_resolution);
            return 0 <= m_y && 0 <= m_x && m_y < m_map.rows&& m_x < m_map.cols;
        }

        void update(const cgaltools::RotatedBox& v_box)
        {
            cv::Point2f points[4];
            v_box.cv_box.points(points);

            auto xmin_point = std::min_element(points, points + 4, [](const cv::Point2f& item1, const cv::Point2f& item2) {return item1.x < item2.x; });
            auto ymin_point = std::min_element(points, points + 4, [](const cv::Point2f& item1, const cv::Point2f& item2) {return item1.y < item2.y; });
            auto xmax_point = std::max_element(points, points + 4, [](const cv::Point2f& item1, const cv::Point2f& item2) {return item1.x < item2.x; });
            auto ymax_point = std::max_element(points, points + 4, [](const cv::Point2f& item1, const cv::Point2f& item2) {return item1.y < item2.y; });

            int xmin = ((*xmin_point).x - m_start[0]) / m_resolution;
            int ymin = ((*ymin_point).y - m_start[1]) / m_resolution;
            int xmax = ((*xmax_point).x - m_start[0]) / m_resolution + 1;
            int ymax = ((*ymax_point).y - m_start[1]) / m_resolution + 1;
            xmin = std::max(xmin, 0);
            xmax = std::min(xmax, m_map.cols - 1);
            ymin = std::max(ymin, 0);
            ymax = std::min(ymax, m_map.rows - 1);
            for (int y = ymin; y <= ymax; ++y)
                for (int x = xmin; x <= xmax; ++x)
                {
                    Eigen::Vector3d point(Eigen::Vector3d(x, y, 0) * m_resolution + Eigen::Vector3d(m_start.x(), m_start.y(), 0));

                    if (v_box.inside_2d(point))
                        m_map.at<float>(y, x) = m_map.at<float>(y, x) > v_box.box.max()[2] ? m_map.at<float>(y, x) : v_box.box.max()[2];
                }

            if (m_dilate != 0)
                cv::dilate(m_map, m_map_dilated,
                    cv::getStructuringElement(cv::MorphShapes::MORPH_RECT, cv::Size(3, 3)),
                    cv::Point(-1, -1),
                    m_dilate);
            else
                m_map_dilated = m_map;

        }

        void save_height_map_png(const std::string& v_path, const float v_threshold = 0.f)
        {
            //std::cout << m_map.rows() << "," << m_map.cols() << std::endl;
            cv::Mat map(m_map.rows, m_map.cols, CV_8UC3);
            map.setTo(cv::Scalar(0, 0, 0));
            for (int y = 0; y < m_map.rows; ++y)
                for (int x = 0; x < m_map.cols; ++x)
                    if (m_map.at<float>(y, x) > v_threshold)
                        map.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            cv::imwrite(v_path, map);
        }

        void save_height_map_tiff(const std::string& v_path) {
            //std::cout << m_map.rows() << "," << m_map.cols() << std::endl;
            cv::imwrite(v_path, m_map);
        }

        void save_height_map_mesh(const fs::path& v_dir) {
            std::vector<Eigen::AlignedBox3d> boxes;
            std::vector<Eigen::AlignedBox3d> boxes_dilated;
            for (int y = 0; y < m_map.rows; ++y)
                for (int x = 0; x < m_map.cols; ++x)
                {
                    Eigen::Vector3d pos(x, y, 0);
                    pos = pos * m_resolution + m_start;
                    if (m_map.at<float>(y, x) > 0)
                        boxes.emplace_back(Eigen::Vector3d(pos.x() - m_resolution / 2, pos.y() - m_resolution / 2, 0.f), Eigen::Vector3d(pos.x() + m_resolution / 2, pos.y() + m_resolution / 2, m_map.at<float>(y, x)));
                    if (m_map_dilated.at<float>(y, x) > 0)
                        boxes_dilated.emplace_back(Eigen::Vector3d(pos.x() - m_resolution / 2, pos.y() - m_resolution / 2, 0.f), Eigen::Vector3d(pos.x() + m_resolution / 2, pos.y() + m_resolution / 2, m_map_dilated.at<float>(y, x)));
                }
            CGAL::IO::write_PLY(((v_dir / "height_map.ply").string()), get_box_mesh(boxes));
            CGAL::IO::write_PLY(((v_dir / "height_map_dialated.ply").string()), get_box_mesh(boxes_dilated));
        }

        void setup_polygon_boundary(const std::vector<Point2>& v_boundary_points)
        {
            m_boundary = Polygon2(v_boundary_points.begin(), v_boundary_points.end());
            has_polygon_boundary = true;
        }

        void setup_strict_polygon_boundary(const std::vector<Point2>& v_boundary_points)
        {
            m_strict_boundary = Polygon2(v_boundary_points.begin(), v_boundary_points.end());
            has_strict_polygon_boundary = true;
        }

        float m_default_height;
        int m_dilate;
        Eigen::Vector3d m_start;
        float m_resolution;
        cv::Mat m_map;
        cv::Mat m_map_dilated;
        Polygon2 m_boundary;
        Polygon2 m_strict_boundary;
        bool has_polygon_boundary = false;
        bool has_strict_polygon_boundary = false;

    };

}

#endif // MODEL_TOOLS_H
