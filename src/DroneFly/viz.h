#pragma once
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/geometry/glgeometry.h>
#include <pangolin/geometry/geometry_ply.h>
#include <pangolin/scene/renderable.h>
#include <thread>
#include <mutex>

#include "building.h"

struct PANGOLIN_EXPORT MyHandler : pangolin::Handler3D
{
    MyHandler(pangolin::OpenGlRenderState& cam_state,
        pangolin::AxisDirection enforce_up = pangolin::AxisNone,
        float trans_scale = 0.01f, float zoom_fraction = PANGO_DFLT_HANDLER3D_ZF
    ) : Handler3D(cam_state, enforce_up, trans_scale, zoom_fraction) {}
};

enum class RenderMode { uv = 0, tex, color, normal, matcap, vertex, num_modes };

const std::string mode_names[] =
{
	"SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP", "SHOW_VERTEX"
};

extern const std::string default_model_shader;

class tVisualizer
{
public:
    std::thread* m_thread;
    std::mutex m_mutex;
    std::string m_model_path;
    pangolin::Geometry oriModel;
    pangolin::OpenGlRenderState s_cam;

    std::vector<Building> m_buildings;
    int m_current_building;
    std::vector<MyViewpoint> m_trajectories;
    std::vector<int> m_is_reconstruction_status;
    std::vector<std::pair<Eigen::Vector2d, cv::Vec3b>> m_uncertainty_map;
    double m_uncertainty_map_distance;
    Eigen::Vector3d m_pos;
    Eigen::Vector3d m_direction;
    PointSet3 m_points;
    std::vector<Eigen::Vector4d> m_points_color;

    tVisualizer(const std::string& v_model_path);

    void draw_point_cloud(
        const PointSet3& v_points
    ); 

    void draw_line(
        const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max, float sickness = 1.,
        const Eigen::Vector4d& v_color = Eigen::Vector4d(1., 0., 0., 1.)
    );

    void draw_cube(
        const cgaltools::RotatedBox& box,
        const Eigen::Vector4d& v_color = Eigen::Vector4d(1., 0., 0., 1.)
    );

    void draw_obj(
        const pangolin::Geometry& v_geometry
    );

    void run();

    void lock()
    {
        m_mutex.lock();
    }

    void unlock() {
        m_mutex.unlock();
    }
};

class oriVisualizer
{
public:
    std::thread* m_thread;
    std::mutex m_mutex;
    std::vector<Building> m_buildings;
    int m_current_building;
    std::vector<MyViewpoint> m_trajectories;
    std::vector<int> m_is_reconstruction_status;
    //std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> m_trajectories_spline;
    std::vector<std::pair<Eigen::Vector2d, cv::Vec3b>> m_uncertainty_map;
    float m_uncertainty_map_distance;
    Eigen::Vector3d m_pos;
    Eigen::Vector3d m_direction;
    PointSet3 m_points;
    std::vector<Eigen::Vector4d> m_points_color;
    //CGAL::General_polygon_2<K> m_polygon;
	
    oriVisualizer()
	{
        m_thread = new std::thread(&oriVisualizer::run, this);
        return;
	}

    void draw_point_cloud(
        const PointSet3& v_points
    );

	
    void draw_line(
        const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max, int sickness = 1,
        const Eigen::Vector4d& v_color = Eigen::Vector4d(1., 0., 0., 1.)
    )
    {
        glLineWidth(static_cast<GLfloat>(sickness));
        glColor3d(static_cast<GLdouble>(v_color.x()), static_cast<GLdouble>(v_color.y()), static_cast<GLdouble>(v_color.z()));
        pangolin::glDrawLine(v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
    }
	
    void draw_cube(
        const cgaltools::RotatedBox& box,
        const Eigen::Vector4d& v_color = Eigen::Vector4d(1., 0., 0., 1.)
    );

    void run(); 

	void lock()
	{
        m_mutex.lock();
	}

    void unlock() {
        m_mutex.unlock();
    }
};