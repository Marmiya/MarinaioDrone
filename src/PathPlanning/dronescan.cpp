#include "dronescan.h"
#include "initialize.h"

std::vector<Viewpoint> droneScan(
    PointSet3& points, const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
    const int maxIterTimes, const int initViewNum, const std::string logPath,
    const double viewDis, const double maxAngle, const double maxDis,
    const bool& modl
)
{
    auto embree_scene = intersectiontools::generate_embree_scene(mesh);
    modeltools::Height_map height_map;
    // Height map generation
	{
	    double area = CGAL::Polygon_mesh_processing::area(mesh);
	    int num_sample = static_cast<int>(area / std::sqrt(3));
	    PointSet3 dense_points_used_for_heightmap_generation;
	    CGAL::Random_points_in_triangle_mesh_3<SurfaceMesh> dense_point_generator(mesh);
		std::copy_n(dense_point_generator, num_sample, dense_points_used_for_heightmap_generation.point_back_inserter());
	    height_map = modeltools::Height_map(dense_points_used_for_heightmap_generation,3,2,6,0);
        height_map.save_height_map_mesh(logPath);
        CGAL::IO::write_point_set(logPath + "dense_sample_points.ply", dense_points_used_for_heightmap_generation);
        LOG(INFO) << boost::format("Generate height map done using %d sample points. Size: %d/%d") % num_sample % height_map.m_map.rows % height_map.m_map.cols;    
    }
	// The threshold of reconstructability which means baseline.
    double tsw1 = 1 / (1 + exp(-32 * (maxAngle * M_PI / 180 - M_PI / 16)));
    double tsw2 = 1 - std::min(viewDis / maxDis, 1.0);
    double tsw3 = 1 - 1 / (1 + exp(-8 * (maxAngle * M_PI / 180 - M_PI_4)));
    double conBaseLine = tsw1 * tsw2 * tsw3 * cos(8.66 * M_PI / 180);

    // The trajectory which is processed in the workflow below.
    std::vector<Viewpoint> trajectory;
    // The final view set.
    std::vector<Viewpoint> finalAns;

    // Size of pointset and viewset.
    int pointsSize = static_cast<int>(points.size());
    int viewSize = static_cast<int>(trajectory.size());

    // Some variable for debugging.
    // *******************************************************************************************
    //std::vector<std::pair<PointSet3, std::vector<Viewpoint>>> one2One(points.size());
    // *******************************************************************************************

    PointSet3 points_failed_to_initialized;
    std::tie(trajectory, points_failed_to_initialized) = initialize_viewpoints(
        points, mesh, embree_scene, height_map, intrinsicMatrix, maxDis,
        viewDis, maxIterTimes, initViewNum
    );

    LOG(INFO) << "Finish initialization of view.";

    // *********************************************************************
    // auto one2Oneitr = std::remove_if(
    //	one2One.begin(), one2One.end(), [](const std::pair<PointSet3, std::vector<Viewpoint>>& a)
    //	{
    //		return a.first.point(0) == Point_3(0, 0, 0);
    //	});
    // one2One = std::move(std::vector<std::pair<PointSet3, std::vector<Viewpoint>>>(one2One.begin(), one2Oneitr));
    //LOG(INFO) << "One2One's size: " << one2One.size();
    // *********************************************************************
    points.collect_garbage();
    CGAL::IO::write_point_set(logPath + "monitoredpts.ply", points);

    pointsSize = static_cast<int>(points.size());
    viewSize = static_cast<int>(trajectory.size());

    LOG(INFO) << "ptssize: " << pointsSize << "\t" << "viewsize: " << viewSize;

    // Selection of views.

    // The reconstructability of every points.
    std::vector<double> recpts(pointsSize, 0);
  
    // Whether a view can watch a points.
    auto visibilityindex = compute_visibilityIndex(trajectory, mesh, points, intrinsicMatrix, maxDis, embree_scene);
    LOG(INFO) << "Finish first vis";

    // ************************************************************************

    // The points and the views which can watch them.
    std::ofstream ffile("C:/Marinaio/TEMP/test1.txt");
    // The points' profile.
    std::ofstream sfile("C:/Marinaio/TEMP/test2.txt");
    // The points' profile after eliminating.
    std::ofstream tfile("C:/Marinaio/TEMP/test3.txt");
    // The importance of views.
    std::ofstream frfile("C:/Marinaio/TEMP/test4.txt");
    // Eliminating views inforamtion.
    std::ofstream fifile("C:/Marinaio/TEMP/test5.txt");
    // The point and its original view(s).
    std::vector<std::pair<PointSet3, std::vector<Viewpoint>>> infooo;
    // The locked views.
    std::vector<int> locked(viewSize, 0);
    // A pointset whose points have low REC.
    PointSet3 low;
    std::vector<std::vector<Viewpoint>> losViews;
    auto lowColorMap = low.add_property_map("color", Eigen::Vector3d(1., 0., 0.)).first;
    // ************************************************************************

    // initialize the reconstructability for every point.
	#pragma omp parallel for
    
    for (int k = 0; k < pointsSize; ++k)
    {
        if (!modl)
        {
            if ((1 + k) % 1000 == 0)
            {
                LOG(INFO) << (k + 1);
            }
        }
        const Eigen::Vector3d point = cgaltools::cgal_point_2_eigen(points.point(k));
        const Eigen::Vector3d normal = cgaltools::cgal_vector_2_eigen(points.normal(k));

        double recp = 0.0;
        const int sz = visibilityindex.at(k).size();

        for (int i = 0; i < sz; ++i)
        {
            Viewpoint l = trajectory.at(visibilityindex.at(k).at(i));

            for (int j = i + 1; j < sz; ++j)
            {
                Viewpoint r = trajectory.at(visibilityindex.at(k).at(j));
                /*double curRec = get<4>(calculate_point_reconstructability(
                    l, r, point, normal, maxDis));*/
                recp += biviewsREC(l.pos_mesh, r.pos_mesh, point, normal, maxDis);
            }
        }

            recpts[k] = recp;
    }
    LOG(INFO) << "Initailzation of pointREC was finished.";

    // ************************************************************************
    //	for (int i = 0; i != pointsSize; i++)
    //	{
    //		std::pair < PointSet3, std::vector<Viewpoint>> t;
    //		t.first.insert(points.point(i));
    //		ffile << "I'm " << i << "th point. \n";
    //		int count = 0;
    //		for (int j = 0; j != viewSize; j++)
    //		{
    //			if (visibility.at(i).at(j))
    //			{
    //				count++;
    //				t.second.push_back(trajectory.at(j));
    //				ffile << trajectory.at(j).pos_mesh.transpose() << " " << trajectory.at(j).direction.transpose() << std::endl;
    //			}
    //		}
    //		infooo.push_back(t);
    //		sfile
    //			<< "I'm " << i << "th point. "
    //			<< "My corrdinate is " << points.point(i) << ". "
    //			<< "My normal is " << points.normal(i) << ".\n"
    //			<< count << " views can see me. "
    //			<< "My REC is " << recpts.at(i) << "\n";
    //	}
    //	LOG(INFO) << "Analysis of pts REC was finished.";
    //
    //#pragma omp parallel for
    //	for (int i = 0; i < pointsSize; i++)
    //	{
    //		if (recpts.at(i) < baseLine) {
    //			for (int j = 0; j != viewSize; j++)
    //			{
    //				if (visibility.at(i).at(j))
    //				{
    //#pragma omp critical
    //					{
    //						locked.at(j) = 1;
    //					}
    //				}
    //			}
    //		}
    //	}
    //
    //	LOG(INFO) << "The amount of locked views is: " << std::reduce(locked.begin(), locked.end());
    // ************************************************************************

    //print_vector_distribution(recpts);

    int del = 0;
	#pragma omp parallel for
    for (int i = 0; i < pointsSize; i++)
    {
        if (recpts.at(i) <= conBaseLine)
        {
			#pragma omp critical
            {
                points.remove(i);
                low.insert(points.point(i));
                del++;
            }
        }
    }

    LOG(INFO) << "Deleted " << del << " points";
    points.collect_garbage();
    pointsSize = static_cast<int>(points.size());
    LOG(INFO) << "Points num now: " << points.size();

    finalAns = droneScanAdj(points, trajectory, mesh, intrinsicMatrix,
        viewDis, maxAngle, maxDis, modl
    );

    

    return finalAns;
}


std::vector<Viewpoint> droneScanAdj(
    const PointSet3& points, const std::vector<Viewpoint>& traj,
    const SurfaceMesh& mesh, const Eigen::Matrix3d& intrinsicMatrix,
    const double viewDis, const double maxAngle, const double maxDis,
    const bool& modl
)
{
    auto embree_scene = intersectiontools::generate_embree_scene(mesh);
    std::vector<Viewpoint> finalAns;

    // The threshold of reconstructability which means baseline.
    double tsw1 = 1 / (1 + exp(-32 * (maxAngle * M_PI / 180 - M_PI / 16)));
    double tsw2 = 1 - std::min(viewDis / maxDis, 1.0);
    double tsw3 = 1 - 1 / (1 + exp(-8 * (maxAngle * M_PI / 180 - M_PI_4)));
    double conBaseLine = tsw1 * tsw2 * tsw3 * cos(8.66 * M_PI / 180);
    double baseLine = 10 * conBaseLine;

    // Size of pointset and viewset.
    const int pointsSize = static_cast<int>(points.size());
    int viewSize = static_cast<int>(traj.size());

    LOG(INFO) << "ptssize: " << pointsSize << "\t" << "viewsize: " << viewSize;

    // The reconstructability of every points.
    std::vector<double> recpts(pointsSize, 0);
    // The indices of the points which a view could watch.
    std::vector<std::vector<int>> correlation(viewSize);
    // The importance of every view
    std::vector<std::pair<double, bool>> importance(viewSize, { 0, true });
    // Whether a view can watch a points.
    auto visibilityindex = compute_visibilityIndex(traj, mesh, points, intrinsicMatrix, maxDis, embree_scene);

    // initialize the reconstructability for every point.
	#pragma omp parallel for
    for (int k = 0; k < pointsSize; ++k)
    {
        Eigen::Vector3d point = cgaltools::cgal_point_2_eigen(points.point(k));
        Eigen::Vector3d normal = cgaltools::cgal_vector_2_eigen(points.normal(k));

        double recp = 0.0;
        std::vector<int> vis = visibilityindex.at(k);

        for (int i = 0; i < vis.size(); ++i)
        {
            Viewpoint l = traj.at(vis.at(i));

            for (int j = i + 1; j < vis.size(); ++j)
            {
                Viewpoint r = traj.at(vis.at(j));
                recp += biviewsREC(l.pos_mesh, r.pos_mesh, point, normal, maxDis);
            }
        }
        recpts[k] = recp;
    }
    LOG(INFO) << "Initailzation of pointREC was finished.";

    std::vector<std::vector<std::pair<int, bool>>> IRD(pointsSize);

	#pragma omp parallel for
    for (int k = 0; k < pointsSize; ++k)
    {
        std::vector<int> vis = visibilityindex.at(k);

        for (int i = 0; i < vis.size(); ++i)
        {
	        #pragma omp critical
            {
                correlation.at(vis.at(i)).push_back(k);
            }

        }
    }
    LOG(INFO) << "Analysis of correlation was finished.";

	#pragma omp parallel for
    for (int i = 0; i < viewSize; ++i)
    {
        double imp = .0;
        auto pset = correlation.at(i);
        for (int j = 0; j < pset.size(); j++)
        {
            int pt = pset.at(j);
            if (recpts.at(pt) - 0. > 1e-6)
            {
                if (1.0 / recpts.at(pt) > imp)
                {
                    imp = 1.0 / recpts.at(pt);
                }
            }
        }
		#pragma omp critical
        {
            importance.at(i).first = imp;
        }
    }
    LOG(INFO) << "Computation of importance is finished.";

	#pragma omp parallel for
    for (int i = 0; i < pointsSize; i++)
    {
        std::vector<std::pair<int, bool>> t;
        auto vis = visibilityindex.at(i);
        for (int j = 0; j < vis.size(); j++)
        {
            t.push_back({ vis.at(j), true });
        }
	    #pragma omp critical
        {
            IRD.at(i) = t;
        }
    }

    LOG(INFO) << "IRD was finised";

  /*  std::ofstream qffile("C:/Marinaio/TEMP/test1232.txt");
    for (int i = 0; i < visibilityindex.size(); i++)
    {
        qffile << visibilityindex.at(i).size() << std::endl;
    }
    std::ofstream qqffile("C:/Marinaio/TEMP/test12qwqw32.txt");
    for (int i = 0; i < IRD.size(); i++)
    {
        qqffile << IRD.at(i).size()<< std::endl;
    }*/


    // Loop for eliminating redundant views.
    int z = viewSize;
    int t = 0;
    while (z > 0)
    {
        auto min = std::min_element(
            importance.begin(), importance.end(),
            [](const std::pair<double, bool>& a, const std::pair<double, bool>& b)
            {
                return a.second ? b.second ? a.first < b.first : true : false;
            });
        int minIndex = min - importance.begin();
      
        auto pts = correlation.at(minIndex);
        std::vector<std::pair<int, double>> storage(pts.size());
        bool remove = true;

		#pragma omp parallel for
        for (int i = 0; i < pts.size(); i++)
        {
            auto pt = pts.at(i);
            Eigen::Vector3d point = cgaltools::cgal_point_2_eigen(points.point(pt));
            Eigen::Vector3d normal = cgaltools::cgal_vector_2_eigen(points.normal(pt));
            double down = 0.0;
            auto& ird = IRD.at(pt);
            int ct = 0;
            int me = 0;

            for (int i = 0; i < ird.size(); i++)
            {
                if (ird.at(i).first == minIndex)
                {
                    me = i;
                }
                else if (ird.at(i).second)
                {
                    double curREC = get<4>(calculate_point_reconstructability(
                        traj.at(minIndex), traj.at(ird.at(i).first), point, normal, maxDis));
                    down += curREC;
                    ct++;
                }
            }
			#pragma omp critical
            {
                storage.at(i) = { me, down };
            }

            if (recpts.at(pt) - down < baseLine && down > recpts.at(pt) * 0.05 || down > recpts.at(pt) * 0.35 || recpts.at(pt) - down < 2)
            {
                remove = false;
            }
        }
       
        min->second = false;
        if (remove)
        {
			#pragma omp parallel for
            for (int i = 0; i < pts.size(); i++)
            {
                auto pt = pts.at(i);
                auto& ird = IRD.at(pt);
				#pragma omp critical
                {
                    recpts.at(pt) -= storage.at(i).second;
                    ird.at(storage.at(i).first).second = false;
                }

                for (int j = 0; j < ird.size(); j++)
                {
                    if (ird.at(j).first != minIndex && importance.at(ird.at(j).first).second)
                    {
                        if (1. / recpts.at(pt) > importance.at(ird.at(j).first).first)
                        {
							#pragma omp critical
                            {
                                importance.at(ird.at(j).first).first = 1. / recpts.at(pt);
                            }
                        }
                    }
                }
            }
            t++;
        }
        else
        {
            finalAns.push_back(traj.at(minIndex));
        }
        z -= 1;
    }

    LOG(INFO) << "Deleted " << t << " views.";
    LOG(INFO) << "Eliminating was finised";
    LOG(INFO) << "There are " << finalAns.size() << " views now.";

    return finalAns;
}