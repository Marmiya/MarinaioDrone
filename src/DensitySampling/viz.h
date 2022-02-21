#pragma once

#include "viz_base.h"

class My_Visualizer: public Visualizer
{
public:
	PointSet3 m_important_points;
	PointSet3 m_unimportant_points;

	My_Visualizer(const std::string& v_model_path):Visualizer(v_model_path)
	{
		m_thread = new std::thread(&My_Visualizer::run, this);
		//render_loop.join();
		return;
	}

	void run()
	{
		auto proxy_model = pangolin::LoadGeometryPly(m_model_path);
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

		pangolin::Var<int> slide_point_size("ui.size",1, 0, 15);
		pangolin::Var<bool> checkbox_mesh("ui.mesh", true, true);
		pangolin::Var<bool> checkbox_important_points("ui.important", false, true);
		pangolin::Var<bool> checkbox_unimportant_points("ui.unimportant", false, true);
		
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
			if (checkbox_important_points)
				draw_point_cloud(m_important_points);
			if (checkbox_unimportant_points)
				draw_point_cloud(m_unimportant_points);

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
