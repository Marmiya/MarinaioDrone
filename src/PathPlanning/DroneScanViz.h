#pragma once
#include "viz_base.h"

using Segment3 = K::Segment_3;

class DroneScanViz: public Visualizer
{
public:
	// Original points.
	PointSet3 sample_points;
	// Initial views.
	std::vector<Viewpoint> initial_views;
	std::vector<Viewpoint> final_views;
	PointSet3 points_failed_to_initialized;
	PointSet3 initial_reconstructability_points;
	PointSet3 final_reconstructability_points;

	DroneScanViz(const std::string& v_model_path):Visualizer(v_model_path)
	{
		m_thread = new std::thread(&DroneScanViz::run, this);
		//render_loop.join();
		return;
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
		                         .SetBounds(0.0, 1.0, pangolin::Attach::Pix(180), 1.0f, -2000.f / 1600.f)
		                         .SetHandler(&handler1);

		pangolin::CreatePanel("ui")
			.SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(180));

		pangolin::Var<int> slide_point_size("ui.size", 1, 0, 15);
		pangolin::Var<bool> checkbox_mesh("ui.mesh", true, true);
		pangolin::Var<bool> checkbox_points("ui.points", false, true);
		pangolin::Var<bool> checkbox_initial_views("ui.init_view", false, true);
		pangolin::Var<bool> checkbox_final_views("ui.final_view", false, true);
		pangolin::Var<bool> checkbox_points_failed_to_initialized("ui.failed_point", false, true);
		pangolin::Var<bool> checkbox_final_reconstructability_points("ui.result_points", false, true);
		pangolin::Var<bool> checkbox_initial_reconstructability_points("ui.init_points", false, true);

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
			if (checkbox_points)
				draw_point_cloud(sample_points);
			if (checkbox_initial_views)
				draw_viewpoint(initial_views,1);
			if (checkbox_final_views)
				draw_viewpoint(final_views,1);
			if(checkbox_final_reconstructability_points)
				draw_point_cloud(final_reconstructability_points);
			if(checkbox_initial_reconstructability_points)
				draw_point_cloud(initial_reconstructability_points);
			if(checkbox_points_failed_to_initialized)
				draw_point_cloud(points_failed_to_initialized);
			m_render_function();

			unlock();

			// Swap frames and Process Events
			pangolin::FinishFrame();
		}

		// unset the current context from the main thread
		pangolin::GetBoundWindow();
	}

};
