#pragma once
//#define __cplusplus 201703L

#include "trajectory.h"
#include "cgal_tools.h"

#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
#include <pangolin/geometry/glgeometry.h>
#include <pangolin/geometry/geometry_ply.h>
#include <pangolin/scene/renderable.h>

#include <thread>
#include <mutex>


namespace pangolin
{
	enum class RenderMode { uv = 0, tex, color, normal, matcap, vertex, num_modes };

	const std::string mode_names[] = {
		"SHOW_UV", "SHOW_TEXTURE", "SHOW_COLOR", "SHOW_NORMAL", "SHOW_MATCAP", "SHOW_VERTEX"
	};

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
}

class Visualizer
{
public:
	pangolin::OpenGlRenderState s_cam;
	std::thread* m_thread;
	std::mutex m_mutex;

	int m_point_size=1;
	std::string m_model_path;

	std::function<void()> m_render_function = []()
	{
	};

	Visualizer(const std::string& v_model_path): m_model_path(v_model_path)
	{
		//m_thread = new std::thread(&Visualizer::run, this);
		return;
	}
	~Visualizer(){
		m_thread->detach();
	}

	void draw_point_cloud(const PointSet3& v_points)
	{
		bool has_color = false;
		if (v_points.has_property_map<Eigen::Vector3d>("color"))
		{
			has_color = true;
		}
		glPointSize(static_cast<GLfloat>(m_point_size));
		glBegin(GL_POINTS);
		const auto& colors = v_points.property_map<Eigen::Vector3d>("color").first;
		for (size_t i = 0; i < v_points.size(); ++i)
		{
			if (has_color)
				glColor3d(colors[i].x(), colors[i].y(), colors[i].z());
			else
				glColor3d(0., 0., 0.);
			glVertex3d(v_points.point(i).x(), v_points.point(i).y(), v_points.point(i).z());
		}
		glEnd();
	}

	void draw_line(const Eigen::Vector3d& v_min, const Eigen::Vector3d& v_max, int sickness = 1,
	               const Eigen::Vector4f& v_color = Eigen::Vector4f(1., 0., 0., 1.))
	{
		glLineWidth(static_cast<GLfloat>(sickness));
		glColor3d(v_color.x(), v_color.y(), v_color.z());
		pangolin::glDrawLine(v_min.x(), v_min.y(), v_min.z(), v_max.x(), v_max.y(), v_max.z());
	}

	void draw_cube(const cgaltools::RotatedBox& box, const Eigen::Vector4f& v_color = Eigen::Vector4f(1., 0., 0., 1.))
	{
		cv::Point2f points[4];
		box.cv_box.points(points);
		const Eigen::Vector3d lt(points[0].x, points[0].y, box.box.min().z());
		const Eigen::Vector3d lb(points[1].x, points[1].y, box.box.min().z());
		const Eigen::Vector3d rt(points[2].x, points[2].y, box.box.max().z());
		const Eigen::Vector3d rb(points[3].x, points[3].y, box.box.max().z());
		//const GLfloat verts[] = {
		//    lt[0],l[1],h[2],  h[0],l[1],h[2],  l[0],h[1],h[2],  h[0],h[1],h[2],  // FRONT
		//    l[0],l[1],l[2],  l[0],h[1],l[2],  h[0],l[1],l[2],  h[0],h[1],l[2],  // BACK
		//    l[0],l[1],h[2],  l[0],h[1],h[2],  l[0],l[1],l[2],  l[0],h[1],l[2],  // LEFT
		//    h[0],l[1],l[2],  h[0],h[1],l[2],  h[0],l[1],h[2],  h[0],h[1],h[2],  // RIGHT
		//    l[0],h[1],h[2],  h[0],h[1],h[2],  l[0],h[1],l[2],  h[0],h[1],l[2],  // TOP
		//    l[0],l[1],h[2],  l[0],l[1],l[2],  h[0],l[1],h[2],  h[0],l[1],l[2]   // BOTTOM
		//};

		const std::vector<Eigen::Vector3d> cube_vertices_vector{
			Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()), // 0 Front-top-left
			Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()), // 1 Front-top-right
			Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()), // 2 Front-bottom-left
			Eigen::Vector3d(points[1].x, points[1].y, box.box.min().z()), // 3 Front-bottom-right
			Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()), // 4 Back-bottom-right
			Eigen::Vector3d(points[1].x, points[1].y, box.box.max().z()), // 5 Front-top-right
			Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z()), // 6 Back-top-right
			Eigen::Vector3d(points[0].x, points[0].y, box.box.max().z()), // 7  Front-top-left
			Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()), // 8 Back-top-left
			Eigen::Vector3d(points[0].x, points[0].y, box.box.min().z()), // 9 Front-bottom-left
			Eigen::Vector3d(points[3].x, points[3].y, box.box.min().z()), // 10 Back-bottom-left
			Eigen::Vector3d(points[2].x, points[2].y, box.box.min().z()), // 11 Back-bottom-right
			Eigen::Vector3d(points[3].x, points[3].y, box.box.max().z()), // 12 Back-top-left
			Eigen::Vector3d(points[2].x, points[2].y, box.box.max().z()) // 13 Back-top-right
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

		//glColor4f(v_color[0], v_color[1], v_color[2], v_color[3]);
		//glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
		//glDrawArrays(GL_TRIANGLE_STRIP, 4, 4);

		//glColor4f(v_color[0], v_color[1], v_color[2], v_color[3]);
		//glDrawArrays(GL_TRIANGLE_STRIP, 8, 4);
		//glDrawArrays(GL_TRIANGLE_STRIP, 12, 4);

		//glColor4f(v_color[0], v_color[1], v_color[2], v_color[3]);
		//glDrawArrays(GL_TRIANGLE_STRIP, 16, 4);
		//glDrawArrays(GL_TRIANGLE_STRIP, 20, 4);

		glDisableClientState(GL_VERTEX_ARRAY);

		draw_line(cube_vertices_vector[0], cube_vertices_vector[1], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[2], cube_vertices_vector[3], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[4], cube_vertices_vector[10], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[6], cube_vertices_vector[8], 2, Eigen::Vector4f(0, 0, 0, 1));

		draw_line(cube_vertices_vector[0], cube_vertices_vector[2], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[1], cube_vertices_vector[3], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[4], cube_vertices_vector[6], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[8], cube_vertices_vector[10], 2, Eigen::Vector4f(0, 0, 0, 1));

		draw_line(cube_vertices_vector[0], cube_vertices_vector[8], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[1], cube_vertices_vector[6], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[3], cube_vertices_vector[4], 2, Eigen::Vector4f(0, 0, 0, 1));
		draw_line(cube_vertices_vector[2], cube_vertices_vector[10], 2, Eigen::Vector4f(0, 0, 0, 1));

	}

	void draw_obj(const pangolin::Geometry& v_geometry)
	{
		if(v_geometry.buffers.size()==0)
			return;
		auto geogl = pangolin::ToGlGeometry(v_geometry);

		pangolin::GlSlProgram default_prog;
		//current_mode = mode;
		default_prog.ClearShaders();
		std::map<std::string, std::string> prog_defines;
		for (int i = 0; i < (int)pangolin::RenderMode::num_modes - 1; ++i)
		{
			prog_defines[pangolin::mode_names[i]] = std::to_string((int)pangolin::RenderMode::normal == i);
		}
		default_prog.AddShader(pangolin::GlSlAnnotatedShader, pangolin::default_model_shader, prog_defines);
		default_prog.Link();

		default_prog.Bind();
		default_prog.SetUniform("KT_cw", s_cam.GetProjectionMatrix() * s_cam.GetModelViewMatrix());
		default_prog.SetUniform("T_cam_norm", s_cam.GetModelViewMatrix());
		pangolin::GlDraw(default_prog, geogl, nullptr);
		default_prog.Unbind();
	}

	void draw_viewpoint(const std::vector<Viewpoint>& v_viewpoint, const double v_half_cube_size = 3,
	                    const Eigen::Vector4f& v_color = Eigen::Vector4f(0, 1, 0, 1),
	                    const bool v_is_draw_connection = false)
	{
		for (int i_view = 0; i_view < v_viewpoint.size(); ++i_view)
		{
			draw_cube(Eigen::AlignedBox3d(
				          v_viewpoint[i_view].pos_mesh - Eigen::Vector3d(
					          v_half_cube_size, v_half_cube_size, v_half_cube_size),
				          v_viewpoint[i_view].pos_mesh + Eigen::Vector3d(
					          v_half_cube_size, v_half_cube_size, v_half_cube_size)),
			          v_color);
			Eigen::Vector3d look_at = v_viewpoint[i_view].pos_mesh + v_viewpoint[i_view].direction * 10;
			draw_line(v_viewpoint[i_view].pos_mesh, look_at, 2, v_color);
			if (v_is_draw_connection && i_view > 0)
			{
				draw_line(v_viewpoint[i_view - 1].pos_mesh, v_viewpoint[i_view].pos_mesh, 2, v_color);
			}
		}
	}

	void run()
	{
		auto proxy_model = pangolin::LoadGeometry(m_model_path);
		pangolin::CreateWindowAndBind("Main", 1600, 900);
		glEnable(GL_DEPTH_TEST);
		s_cam = pangolin::OpenGlRenderState(
			pangolin::ProjectionMatrix(1600, 900, 800, 450, 800, 450, 1, 99999),
			pangolin::ModelViewLookAt(40, 40, 40, 0, 0, 0, pangolin::AxisZ)
		);

		// Create Interactive View in window
		pangolin::Handler3D handler1(s_cam, pangolin::AxisZ);
		pangolin::View& d_cam1 = pangolin::CreateDisplay()
		                         .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0, -2000. / 1600.)
		                         .SetHandler(&handler1);

		pangolin::CreatePanel("ui")
			.SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

		pangolin::Var<int> slide_point_size("ui.size",1, 0, 15);
		pangolin::Var<bool> checkbox_mesh("ui.mesh", true, true);
		pangolin::Var<bool> checkbox_sample_points("ui.sample_points", false, true);
		pangolin::Var<bool> checkbox_first_pass_viewpoint("ui.viewpoint", false, true);
		pangolin::Var<bool> checkbox_sample_points_with_reconstructability("ui.points_r", false, true);
		pangolin::Var<double> slide_bar_reconstructability("ui.r",1., 0., 50.);
		pangolin::Var<bool> checkbox_sample_points_with_error("ui.points_e", false, true);
		pangolin::Var<double> slide_bar_max_error("ui.e",0.02f, 0., 1.);
		pangolin::Var<bool> checkbox_sample_points_with_zero_reconstructability("ui.zero_reconstructability", false, true);
		
		while (!pangolin::ShouldQuit())
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glClearColor(0.56f, 0.56f, 0.56f, 1);
			m_point_size = slide_point_size.Get();
			lock();

			d_cam1.Activate(s_cam);
			pangolin::glDrawAxis(1000);
			// Render
			if (checkbox_mesh)
				draw_obj(proxy_model);

			m_render_function();

			unlock();

			// Swap frames and Process Events
			pangolin::FinishFrame();
		}

		// unset the current context from the main thread
		pangolin::GetBoundWindow();
	}

	void lock()
	{
		m_mutex.lock();
	}

	void unlock()
	{
		m_mutex.unlock();
	}
};
