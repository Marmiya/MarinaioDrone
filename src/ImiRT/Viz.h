#pragma once
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/geometry/glgeometry.h>
#include <pangolin/geometry/geometry_ply.h>
#include <pangolin/scene/renderable.h>
#include <thread>
#include <mutex>
#include <model_tools.h>

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
    std::thread* thread;
    std::mutex mutex;

    pangolin::Geometry oriModel;
    pangolin::OpenGlRenderState cam;

    Eigen::Vector3d pos;
    Eigen::Vector3d direction;
    PointSet3 pts;
    std::vector<Eigen::Vector4d> m_points_color;

    tVisualizer(const std::string& modelPath);

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
        mutex.lock();
    }

    void unlock()
    {
        mutex.unlock();
    }
};