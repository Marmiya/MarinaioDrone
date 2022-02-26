#pragma once

#include "viz_base.h"

#include "intersection_tools.h"

using Segment3 = K::Segment_3;

std::vector<Viewpoint>
watchingView(
	const Tree& tree, const std::vector<Viewpoint>& traj,
	const Point3& p, const double& viewDis)
{
	std::vector<Viewpoint> out;
#pragma omp parallel for
	for (int i = 0; i < static_cast<int>(traj.size()); ++i)
	{
		Viewpoint vp = traj.at(i);
		Point3 base = cgaltools::eigen_2_cgal_point(vp.pos_mesh);
		auto direction = cgaltools::eigen_2_cgal_vector(vp.direction);
		
		const double dis = sqrt(CGAL::squared_distance(p, base));
		if (dis > viewDis - 0.1 && dis < viewDis + 0.1)
		{
			Vector3 viewseg(base, p);
			Vector3 unitViewSeg = viewseg / sqrt(viewseg.squared_length());
			viewseg = unitViewSeg * (viewDis - 0.1);
			Point3 survey(base + viewseg);
			Segment3 viewSeg(base, survey);

			auto intersection = tree.any_intersection(viewSeg);
			if (!intersection)
			{
				if (double aco = acos(unitViewSeg * direction) < M_PI / 6)
				{
#pragma omp critical
					{
						out.push_back(vp);
					}
				}
			}

		}
	}
	return out;
}

class My_Visualizer: public Visualizer
{
public:
	// Original points.
	PointSet3 sample_points;
	// Initial views.
	std::vector<Viewpoint> sample_views;
	// The points whose REC is larger than a threshold.
	PointSet3 validPts;
	// The final views
	std::vector<Viewpoint> m_viewpoint;
	// The points whose REC is small.
	PointSet3 lowRECPts;
	// AABB tree
	Tree trey;
	// The points and their initial views, one point by five views.
	std::vector<std::pair<PointSet3, std::vector<Viewpoint>>> one2One;
	// The points and the views which can watch them.
	std::vector<std::pair<PointSet3, std::vector<Viewpoint>>> infooo;

	int showdetail;
	int lastdetail;

	int o2o;
	int lasto2o;

	My_Visualizer(const std::string& v_model_path):Visualizer(v_model_path)
	{
		m_thread = new std::thread(&My_Visualizer::run, this);
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
		pangolin::Var<bool> checkbox_sample_points("ui.sample_points", false, true);
		pangolin::Var<bool> lowREC("ui.low REC", false, true);
		pangolin::Var<bool> checkbox_first_pass_viewpoint("ui.viewpoint", false, true);
		pangolin::Var<bool> checkbox_sample_points_with_reconstructability("ui.points_r", false, true);
		pangolin::Var<double> slide_bar_reconstructability("ui.r",1.f, 0.f, 50.f);
		pangolin::Var<bool> checkbox_sample_points_with_zero_reconstructability("ui.zero_reconstructability", false, true);
		pangolin::Var<std::string> showd("ui.sh", "0");
		pangolin::Var<bool> showde("ui.showDET", false, true);
		pangolin::Var<bool> validpts("validpts", false, true);
		pangolin::Var<std::string> o2oinput("ui.o2o", "0");
		pangolin::Var<bool> showo2o("ui.showo2o", false, true);

		while (!pangolin::ShouldQuit())
		{
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
			glClearColor(0.56f, 0.56f, 0.56f, 1);
			m_point_size = slide_point_size.Get();
			showdetail = std::stoi(showd.Get());
			o2o = std::stoi(o2oinput.Get());

			lock();

			d_cam1.Activate(s_cam);
			pangolin::glDrawAxis(1000);
			// Render
			if (checkbox_mesh)
				draw_obj(proxy_model);
			if (checkbox_sample_points)
				draw_point_cloud(sample_points);
			if (validpts)
				draw_point_cloud(validPts);
			if (lowREC)
				draw_point_cloud(lowRECPts);
			if (checkbox_first_pass_viewpoint)
				draw_viewpoint(m_viewpoint, 1);
			if (checkbox_sample_points_with_reconstructability)
			{
				auto reconstructability_map = sample_points.property_map<double>("reconstructability").first;
				if(!sample_points.property_map<double>("reconstructability").second)
				{
					std::cout << "No reconstructability map"<<std::endl;
				}
				else
				{
					glPointSize(m_point_size);
					glBegin(GL_POINTS);
					for (int i_point = 0; i_point < sample_points.size(); ++i_point)
					{
						Eigen::Vector3d color =  std::min(reconstructability_map[i_point],slide_bar_reconstructability.Get())/slide_bar_reconstructability.Get()*Eigen::Vector3d(1,1,1);
						glColor3d(color.x(),color.y(),color.z());
						glVertex3d(sample_points.point(i_point).x(), sample_points.point(i_point).y(), sample_points.point(i_point).z());
					}
					glEnd();
				}
				
			}
			if (checkbox_sample_points_with_zero_reconstructability)
			{
				if(!sample_points.property_map<double>("reconstructability").second)
				{
					std::cout << "No reconstructability map"<<std::endl;
				}
				else
				{
					auto reconstructability_map = sample_points.property_map<double>("reconstructability").first;
					glPointSize(m_point_size);
					glBegin(GL_POINTS);
					glColor3d(0.f, 0.f, 0.f);

					for (int i_point = 0; i_point < sample_points.size(); ++i_point)
						if(reconstructability_map[i_point] == 0)
							glVertex3d(sample_points.point(i_point).x(), sample_points.point(i_point).y(), sample_points.point(i_point).z());
					glEnd();
				}
				
			}
			if(showde)
			{
				draw_point_cloud(infooo.at(showdetail).first);
				draw_viewpoint(infooo.at(showdetail).second, 1);
			}
			if(showo2o)
			{
				/*if (lasto2o != o2o)
				{
					std::cout << one2One.at(o2o).second.size() << std::endl;
				}*/
				draw_point_cloud(one2One.at(o2o).first);
				draw_viewpoint(one2One.at(o2o).second);
			}
			m_render_function();

			lastdetail = showdetail;
			lasto2o = o2o;

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
