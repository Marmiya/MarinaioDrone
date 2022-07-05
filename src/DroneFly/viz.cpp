#include "viz.h"

tVisualizer::tVisualizer(
    const std::string& v_model_path
) : m_model_path(v_model_path)
{
    oriModel = pangolin::LoadGeometry(m_model_path);
    m_thread = new std::thread(&tVisualizer::run, this);
    return;
}

void tVisualizer::draw_point_cloud(
    const PointSet3& v_points
)
{
    glBegin(GL_POINTS);
    glColor3d(1, 1, 1);
    for (size_t i = 0; i < v_points.size(); ++i) {
        glPointSize(5);
        glVertex3d(v_points.point(i).x(), v_points.point(i).y(), v_points.point(i).z());
    }
    glEnd();
    glPointSize(1);
}

void tVisualizer::draw_line(
    const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max, float sickness, const Eigen::Vector4d& v_color
)
{
    glLineWidth(sickness);
    glColor3d(v_color.x(), v_color.y(), v_color.z());
    pangolin::glDrawLine(v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
}

void tVisualizer::draw_cube(
    const cgaltools::RotatedBox& box, const Eigen::Vector4d& v_color
)
{
    cv::Point2f points[4];
    box.cv_box.points(points);
    const Eigen::Vector3d lt(points[0].x, points[0].y, box.box.min().z());
    const Eigen::Vector3d lb(points[1].x, points[1].y, box.box.min().z());
    const Eigen::Vector3d rt(points[2].x, points[2].y, box.box.max().z());
    const Eigen::Vector3d rb(points[3].x, points[3].y, box.box.max().z());


    const std::vector<Eigen::Vector3d> cube_vertices_vector{
        Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()),     // 0 Front-top-left
        Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()),      // 1 Front-top-right
        Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()),    // 2 Front-bottom-left
        Eigen::Vector3d(points[1].x, points[1].y, box.box.min().z()),     // 3 Front-bottom-right
        Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()),    // 4 Back-bottom-right
        Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()),      // 5 Front-top-right
        Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z()),     // 6 Back-top-right
        Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()),     // 7  Front-top-left
        Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()),    // 8 Back-top-left
        Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()),    // 9 Front-bottom-left
        Eigen::Vector3d(points[3].x, points[3].y, box.box.min().z()),   // 10 Back-bottom-left
        Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()),    // 11 Back-bottom-right
        Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()),    // 12 Back-top-left
        Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z())      // 13 Back-top-right
    };

    GLfloat verts[42];
    int idx = 0;
    for (const auto& item : cube_vertices_vector)
    {
        verts[3 * idx + 0] = item.x();
        verts[3 * idx + 1] = item.y();
        verts[3 * idx + 2] = item.z();
        idx += 1;
    }

    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);

    glColor4f(v_color[0], v_color[1], v_color[2], v_color[3]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 14);

    glDisableClientState(GL_VERTEX_ARRAY);

    draw_line(cube_vertices_vector[0], cube_vertices_vector[1], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[2], cube_vertices_vector[3], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[4], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[6], cube_vertices_vector[8], 2, Eigen::Vector4d(0, 0, 0, 1));

    draw_line(cube_vertices_vector[0], cube_vertices_vector[2], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[1], cube_vertices_vector[3], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[4], cube_vertices_vector[6], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[8], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));

    draw_line(cube_vertices_vector[0], cube_vertices_vector[8], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[1], cube_vertices_vector[6], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[3], cube_vertices_vector[4], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[2], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));
}

void tVisualizer::draw_obj(
    const pangolin::Geometry& v_geometry
)
{
    if (v_geometry.buffers.empty())
        return;
    auto geogl = pangolin::ToGlGeometry(v_geometry);

    pangolin::GlSlProgram default_prog;
    //current_mode = mode;
    default_prog.ClearShaders();
    std::map<std::string, std::string> prog_defines;
    for (int i = 0; i < (int)RenderMode::num_modes - 1; ++i)
    {
        prog_defines[mode_names[i]] = std::to_string((int)RenderMode::normal == i);
    }
    default_prog.AddShader(pangolin::GlSlAnnotatedShader, default_model_shader, prog_defines);
    default_prog.Link();

    default_prog.Bind();
    default_prog.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
    default_prog.SetUniform("T_cam_norm", s_cam.GetModelViewMatrix());
    pangolin::GlDraw(default_prog, geogl, nullptr);
    default_prog.Unbind();
}

void tVisualizer::run()
{
    pangolin::CreateWindowAndBind("DFP", 1080, 720);
    glEnable(GL_DEPTH_TEST);
    s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1080, 720, 800, 450, 800, 450, 1, 99999),
        pangolin::ModelViewLookAt(-100, -100, 80, 0, 0, -1, pangolin::AxisZ)
    );

    MyHandler handler1(s_cam, pangolin::AxisZ);

    pangolin::View& d_cam1 = pangolin::CreateDisplay()
        .SetBounds(0., 1., 0., 1., -2000. / 1920.)
        .SetHandler(&handler1);

    while (!pangolin::ShouldQuit())
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(static_cast<GLclampf>(0.56), static_cast<GLclampf>(0.56), static_cast<GLclampf>(0.56), static_cast<GLclampf>(1));

        lock();
        d_cam1.Activate(s_cam);
        for (const auto& item_tile : m_uncertainty_map) {
            const Eigen::Vector2d& position = item_tile.first;
            const cv::Vec3b& item_color = item_tile.second;
            Eigen::Vector4d color;
            color = Eigen::Vector4d(
                static_cast<double>(item_color[2] / 255.), static_cast<double>(item_color[1] / 255.),
                static_cast<double>(item_color[0] / 255.), 1
            );

            draw_cube(Eigen::AlignedBox3d(Eigen::Vector3d(position.x() - m_uncertainty_map_distance / 2, position.y() - m_uncertainty_map_distance / 2, -1),
                Eigen::Vector3d(position.x() + m_uncertainty_map_distance / 2, position.y() + m_uncertainty_map_distance / 2, 1)), color);
        }

        draw_obj(oriModel);

        //View points
        for (const auto& item_trajectory : m_trajectories) 
        {
            draw_cube(Eigen::AlignedBox3d(item_trajectory.pos_mesh - Eigen::Vector3d(1., 1., 1.), item_trajectory.pos_mesh + Eigen::Vector3d(1., 1., 1.)),
                Eigen::Vector4d(0., 1., 0., 1.));
            Eigen::Vector3d look_at = item_trajectory.pos_mesh + item_trajectory.direction * 10;
            draw_line(item_trajectory.pos_mesh, look_at, 2, Eigen::Vector4d(0, 1, 0, 1));
        }
        //View spline
        for (const auto& item_trajectory : m_trajectories)
        {
            int index = &item_trajectory - &m_trajectories[0];

            Eigen::Vector4d color(250. / 255, 157. / 255, 0. / 255, 1);
            if (item_trajectory.is_towards_reconstruction == true)
                color = Eigen::Vector4d(23. / 255, 73. / 255, 179. / 255, 1);

            glColor3d(color.x(), color.y(), color.z());
            if (index >= 1)
                pangolin::glDrawLine(item_trajectory.pos_mesh[0], item_trajectory.pos_mesh[1], item_trajectory.pos_mesh[2],
                    m_trajectories[index - 1].pos_mesh[0], m_trajectories[index - 1].pos_mesh[1], m_trajectories[index - 1].pos_mesh[2]);

            glColor3d(0, 0, 0);
        }

        // Current Position and orientation
        draw_cube(Eigen::AlignedBox3d(m_pos - Eigen::Vector3d(1., 1., 1.), m_pos + Eigen::Vector3d(1., 1., 1.)),
            Eigen::Vector4d(1., 0., 0., 1.));
        Eigen::Vector3d look_at = m_pos + m_direction * 20;
        draw_line(m_pos, look_at, 2, Eigen::Vector4d(0, 1, 0, 1));

        // Uncertainty
        for (const auto& item_tile : m_uncertainty_map) 
        {
            const Eigen::Vector2d& position = item_tile.first;
            const cv::Vec3b& item_color = item_tile.second;
            Eigen::Vector4d color;
            color = Eigen::Vector4d((float)item_color[2] / 255., (float)item_color[1] / 255., (float)item_color[0] / 255., 1);

            draw_cube(Eigen::AlignedBox3d(Eigen::Vector3d(position.x() - m_uncertainty_map_distance / 2, position.y() - m_uncertainty_map_distance / 2, -1),
                Eigen::Vector3d(position.x() + m_uncertainty_map_distance / 2, position.y() + m_uncertainty_map_distance / 2, 1)), color);
        }

        unlock();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    // unset the current context from the main thread
    pangolin::GetBoundWindow();
}

/*
 * Original Vizer
 */

void oriVisualizer::draw_point_cloud(
    const PointSet3& v_points
)
{
    glBegin(GL_POINTS);
    glColor3d(1, 1, 1);
    for (size_t i = 0; i < v_points.size(); ++i) {
        glPointSize(5);

        glVertex3d(v_points.point(i).x(), v_points.point(i).y(), v_points.point(i).z());
    }
    glEnd();
    glPointSize(1);
}

void oriVisualizer::draw_cube(
    const cgaltools::RotatedBox& box, const Eigen::Vector4d& v_color
)
{
    cv::Point2f points[4];
    box.cv_box.points(points);
    const Eigen::Vector3d lt(points[0].x, points[0].y, box.box.min().z());
    const Eigen::Vector3d lb(points[1].x, points[1].y, box.box.min().z());
    const Eigen::Vector3d rt(points[2].x, points[2].y, box.box.max().z());
    const Eigen::Vector3d rb(points[3].x, points[3].y, box.box.max().z());

    const std::vector<Eigen::Vector3d> cube_vertices_vector{
        Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()),     // 0 Front-top-left
        Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()),      // 1 Front-top-right
        Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()),    // 2 Front-bottom-left
        Eigen::Vector3d(points[1].x, points[1].y, box.box.min().z()),     // 3 Front-bottom-right
        Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()),    // 4 Back-bottom-right
        Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()),      // 5 Front-top-right
        Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z()),     // 6 Back-top-right
        Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()),     // 7  Front-top-left
        Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()),    // 8 Back-top-left
        Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()),    // 9 Front-bottom-left
        Eigen::Vector3d(points[3].x, points[3].y, box.box.min().z()),   // 10 Back-bottom-left
        Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()),    // 11 Back-bottom-right
        Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()),    // 12 Back-top-left
        Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z())      // 13 Back-top-right
    };

    GLfloat verts[42];
    int idx = 0;
    for (const auto& item : cube_vertices_vector)
    {
        verts[3 * idx + 0] = item.x();
        verts[3 * idx + 1] = item.y();
        verts[3 * idx + 2] = item.z();
        idx += 1;
    }

    glVertexPointer(3, GL_FLOAT, 0, verts);
    glEnableClientState(GL_VERTEX_ARRAY);

    glColor4f(v_color[0], v_color[1], v_color[2], v_color[3]);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 14);

    glDisableClientState(GL_VERTEX_ARRAY);

    draw_line(cube_vertices_vector[0], cube_vertices_vector[1], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[2], cube_vertices_vector[3], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[4], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[6], cube_vertices_vector[8], 2, Eigen::Vector4d(0, 0, 0, 1));

    draw_line(cube_vertices_vector[0], cube_vertices_vector[2], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[1], cube_vertices_vector[3], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[4], cube_vertices_vector[6], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[8], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));

    draw_line(cube_vertices_vector[0], cube_vertices_vector[8], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[1], cube_vertices_vector[6], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[3], cube_vertices_vector[4], 2, Eigen::Vector4d(0, 0, 0, 1));
    draw_line(cube_vertices_vector[2], cube_vertices_vector[10], 2, Eigen::Vector4d(0, 0, 0, 1));

}

void oriVisualizer::run()
{
    pangolin::CreateWindowAndBind("DFP", 1920, 1080);

    glEnable(GL_DEPTH_TEST);

    pangolin::OpenGlRenderState s_cam1(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 99999),
        pangolin::ModelViewLookAt(40, 40, 40, 0, 0, 0, pangolin::AxisZ)
    );
    pangolin::OpenGlRenderState s_cam2(
        pangolin::ProjectionMatrix(1280, 960, 50, 50, 640, 480, 0.2, 99999),
        pangolin::ModelViewLookAt(-250, 1., 40., -250, 0, 0, pangolin::AxisZ)
    );

    // Create Interactive View in window
    MyHandler handler1(s_cam1, pangolin::AxisZ);
    MyHandler handler2(s_cam2, pangolin::AxisZ);

    pangolin::View& d_cam1 = pangolin::CreateDisplay()
        .SetBounds(0.0, 1., 0., 1., -640. / 480.)
        .SetHandler(&handler1);
    pangolin::View& d_cam2 = pangolin::CreateDisplay()
        .SetBounds(0, 1., 1., 1., -640. / 480.)
        .SetHandler(&handler2);

    pangolin::Display("multi")
        .SetBounds(0.0, 1.0, 0.0, 1.0)
        .SetLayout(pangolin::LayoutEqualHorizontal)
        .AddDisplay(d_cam1)
        .AddDisplay(d_cam2)
        ;

    while (!pangolin::ShouldQuit()) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(static_cast<GLclampf>(0.56), static_cast<GLclampf>(0.56), static_cast<GLclampf>(0.56), static_cast<GLclampf>(1));

        lock();
        d_cam2.Activate(s_cam2);
        for (const auto& item_tile : m_uncertainty_map) {
            const Eigen::Vector2d& position = item_tile.first;
            const cv::Vec3b& item_color = item_tile.second;
            Eigen::Vector4d color;
            color = Eigen::Vector4d(
                static_cast<double>(item_color[2] / 255.), static_cast<double>(item_color[1] / 255.),
                static_cast<double>(item_color[0] / 255.), 1
            );

            draw_cube(Eigen::AlignedBox3d(Eigen::Vector3d(position.x() - m_uncertainty_map_distance / 2, position.y() - m_uncertainty_map_distance / 2, -1),
                Eigen::Vector3d(position.x() + m_uncertainty_map_distance / 2, position.y() + m_uncertainty_map_distance / 2, 1)), color);
        }

        d_cam1.Activate(s_cam1);

        //Building
        for (const auto& item_building : m_buildings)
        {
            if (item_building.is_divide)
                continue;
            int index = &item_building - &m_buildings[0];
            if (m_current_building == index)
                draw_cube(item_building.bounding_box_3d,
                    Eigen::Vector4d(1., 0., 0., 1.));
            else if (!item_building.passed_trajectory.empty())
                draw_cube(item_building.bounding_box_3d,
                    Eigen::Vector4d(1., 1., 1., 1.));
            else
                draw_cube(item_building.bounding_box_3d,
                    Eigen::Vector4d(0.5, .5, .5, .5));
        }

        //View points
        for (const auto& item_trajectory : m_trajectories) {
            draw_cube(Eigen::AlignedBox3d(item_trajectory.pos_mesh - Eigen::Vector3d(1., 1., 1.), item_trajectory.pos_mesh + Eigen::Vector3d(1., 1., 1.)),
                Eigen::Vector4d(0., 1., 0., 1.));
            Eigen::Vector3d look_at = item_trajectory.pos_mesh + item_trajectory.direction * 10;
            draw_line(item_trajectory.pos_mesh, look_at, 2, Eigen::Vector4d(0, 1, 0, 1));
        }
        //View spline
        for (const auto& item_trajectory : m_trajectories) {
            int index = &item_trajectory - &m_trajectories[0];

            Eigen::Vector4d color(250. / 255, 157. / 255, 0. / 255, 1);
            if (item_trajectory.is_towards_reconstruction == true)
                color = Eigen::Vector4d(23. / 255, 73. / 255, 179. / 255, 1);

            //draw_cube(Eigen::AlignedBox3d(item_trajectory.first - Eigen::Vector3d(1., 1., 1.), item_trajectory.first + Eigen::Vector3d(1., 1., 1.)),
            //    Eigen::Vector4d(0., 1., 0., 1.));
            glColor3d(color.x(), color.y(), color.z());
            if (index >= 1)
                pangolin::glDrawLine(item_trajectory.pos_mesh[0], item_trajectory.pos_mesh[1], item_trajectory.pos_mesh[2],
                    m_trajectories[index - 1].pos_mesh[0], m_trajectories[index - 1].pos_mesh[1], m_trajectories[index - 1].pos_mesh[2]);

            glColor3d(0, 0, 0);
        }

        // Current Position and orientation
        draw_cube(Eigen::AlignedBox3d(m_pos - Eigen::Vector3d(4., 4., 4.), m_pos + Eigen::Vector3d(4., 4., 4.)),
            Eigen::Vector4d(1., 0., 0., 1.));
        Eigen::Vector3d look_at = m_pos + m_direction * 20;
        draw_line(m_pos, look_at, 2, Eigen::Vector4d(0, 1, 0, 1));

        // Uncertainty
        for (const auto& item_tile : m_uncertainty_map) {
            const Eigen::Vector2d& position = item_tile.first;
            const cv::Vec3b& item_color = item_tile.second;
            Eigen::Vector4d color;
            color = Eigen::Vector4d((float)item_color[2] / 255., (float)item_color[1] / 255., (float)item_color[0] / 255., 1);

            draw_cube(Eigen::AlignedBox3d(Eigen::Vector3d(position.x() - m_uncertainty_map_distance / 2, position.y() - m_uncertainty_map_distance / 2, -1),
                Eigen::Vector3d(position.x() + m_uncertainty_map_distance / 2, position.y() + m_uncertainty_map_distance / 2, 1)), color);
        }

        unlock();

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }
    // unset the current context from the main thread
    pangolin::GetBoundWindow();
}

const std::string default_model_shader = R"Shader(
/////////////////////////////////////////
@start vertex
#version 120

#expect SHOW_COLOR
#expect SHOW_NORMAL
#expect SHOW_TEXTURE
#expect SHOW_MATCAP
#expect SHOW_UV

    uniform mat4 T_cam_norm;
    uniform mat4 KT_cw;
    attribute vec3 vertex;

#if SHOW_COLOR
    attribute vec4 color;
    varying vec4 vColor;
    void main() {
        vColor = color;
#elif SHOW_NORMAL
    attribute vec3 normal;
    varying vec3 vNormal;
    void main() {
        vNormal = mat3(T_cam_norm) * normal;
#elif SHOW_TEXTURE
    attribute vec2 uv;
    varying vec2 vUV;
    void main() {
        vUV = uv;
#elif SHOW_MATCAP
    attribute vec3 normal;
    varying vec3 vNormalCam;
    void main() {
        vNormalCam = mat3(T_cam_norm) * normal;
#elif SHOW_UV
    attribute vec2 uv;
    varying vec2 vUV;
    void main() {
        vUV = uv;
#else
    varying vec3 vP;
    void main() {
        vP = vertex;
#endif
        gl_Position = KT_cw * vec4(vertex, 1.0);
    }

/////////////////////////////////////////
@start fragment
#version 120
#expect SHOW_COLOR
#expect SHOW_NORMAL
#expect SHOW_TEXTURE
#expect SHOW_MATCAP
#expect SHOW_UV

#if SHOW_COLOR
    varying vec4 vColor;
#elif SHOW_NORMAL
    varying vec3 vNormal;
#elif SHOW_TEXTURE
    varying vec2 vUV;
    uniform sampler2D texture_0;
#elif SHOW_MATCAP
    varying vec3 vNormalCam;
    uniform sampler2D matcap;
#elif SHOW_UV
    varying vec2 vUV;
#else
    varying vec3 vP;
#endif

void main() {
#if SHOW_COLOR
    gl_FragColor = vColor;
#elif SHOW_NORMAL
    gl_FragColor = vec4((vNormal + vec3(1.0,1.0,1.0)) / 2.0, 1.0);
#elif SHOW_TEXTURE
    gl_FragColor = texture2D(texture_0, vUV);
#elif SHOW_MATCAP
    vec2 uv = 0.5 * vNormalCam.xy + vec2(0.5, 0.5);
    gl_FragColor = texture2D(matcap, uv);
#elif SHOW_UV
    gl_FragColor = vec4(vUV,1.0-vUV.x,1.0);
#else
    gl_FragColor = vec4(vP / 100.0,1.0);
#endif
}
)Shader";

const std::string equi_env_shader = R"Shader(
/////////////////////////////////////////
@start vertex
#version 120
attribute vec2 vertex;
attribute vec2 xy;
varying vec2 vXY;

void main() {
    vXY = xy;
    gl_Position = vec4(vertex,0.0,1.0);
}

@start fragment
#version 120
#define M_PI 3.1415926538
uniform sampler2D texture_0;
uniform mat3 R_env_camKinv;
varying vec2 vXY;

vec2 RayToEquirect(vec3 ray)
{
    double n = 1.0;
    double m = 1.0;
    double lamda = acos(ray.y/sqrt(1.0-ray.z*ray.z));
    if(ray.x < 0) lamda = -lamda;
    double phi = asin(ray.z);
    double u = n*lamda/(2.0*M_PI)+n/2.0;
    double v = m/2.0 + m*phi/M_PI;
    return vec2(u,v);
}

void main() {
    vec3 ray_env = normalize(R_env_camKinv * vec3(vXY, 1.0));
    gl_FragColor = texture2D(texture_0, RayToEquirect(ray_env));
}
)Shader";