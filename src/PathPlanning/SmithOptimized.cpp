#include "SmithOptimized.h"

enum ompStatus
{
    init,
	good,
    slow,
    failure,
    unenough
};

double harmony = 0.3;
double hallmark = 20.;
double upperHallmark = 20.;
double steplength = 4.;
int outerIternum = 2;
int innerItrnum = 50;
/*
std::array<std::pair<double, double>, 128> viewCandidates{
   {
	{9. / 16. * M_PI,0.},{10. / 16. * M_PI,0.},{11. / 16. * M_PI,0.},{12. / 16. * M_PI,0.},
    {13. / 16. * M_PI,0.},{14. / 16. * M_PI,0.},{15. / 16. * M_PI,0.},{M_PI,0.},
    {9. / 16. * M_PI,1. / 8. * M_PI},{10. / 16. * M_PI,1. / 8. * M_PI},{11. / 16. * M_PI,1. / 8. * M_PI},{12. / 16. * M_PI,1. / 8. * M_PI},
    {13. / 16. * M_PI,1. / 8. * M_PI},{14. / 16. * M_PI,1. / 8. * M_PI},{15. / 16. * M_PI,1. / 8. * M_PI},{M_PI,1. / 8. * M_PI},
    {9. / 16. * M_PI,2. / 8. * M_PI},{10. / 16. * M_PI,2. / 8. * M_PI},{11. / 16. * M_PI,2. / 8. * M_PI},{12. / 16. * M_PI,0.},
    {13. / 16. * M_PI,2. / 8. * M_PI},{14. / 16. * M_PI,2. / 8. * M_PI},{15. / 16. * M_PI,2. / 8. * M_PI},{M_PI,2. / 8. * M_PI},
    {9. / 16. * M_PI,3. / 8. * M_PI},{10. / 16. * M_PI,3. / 8. * M_PI},{11. / 16. * M_PI,3. / 8. * M_PI},{12. / 16. * M_PI,3. / 8. * M_PI},
    {13. / 16. * M_PI,3. / 8. * M_PI},{14. / 16. * M_PI,3. / 8. * M_PI},{15. / 16. * M_PI,3. / 8. * M_PI},{M_PI,3. / 8. * M_PI},
    {9. / 16. * M_PI,4. / 8. * M_PI},{10. / 16. * M_PI,4. / 8. * M_PI},{11. / 16. * M_PI,4. / 8. * M_PI},{12. / 16. * M_PI,4. / 8. * M_PI},
    {13. / 16. * M_PI,4. / 8. * M_PI},{14. / 16. * M_PI,4. / 8. * M_PI},{15. / 16. * M_PI,4. / 8. * M_PI},{M_PI,4. / 8. * M_PI},
    {9. / 16. * M_PI,5. / 8. * M_PI},{10. / 16. * M_PI,5. / 8. * M_PI},{11. / 16. * M_PI,5. / 8. * M_PI},{12. / 16. * M_PI,5. / 8. * M_PI},
    {13. / 16. * M_PI,5. / 8. * M_PI},{14. / 16. * M_PI,5. / 8. * M_PI},{15. / 16. * M_PI,5. / 8. * M_PI},{M_PI,5. / 8. * M_PI},
    {9. / 16. * M_PI,6. / 8. * M_PI},{10. / 16. * M_PI,6. / 8. * M_PI},{11. / 16. * M_PI,6. / 8. * M_PI},{12. / 16. * M_PI,6. / 8. * M_PI},
    {13. / 16. * M_PI,6. / 8. * M_PI},{14. / 16. * M_PI,6. / 8. * M_PI},{15. / 16. * M_PI,6. / 8. * M_PI},{M_PI,6. / 8. * M_PI},
    {9. / 16. * M_PI,7. / 8. * M_PI},{10. / 16. * M_PI,7. / 8. * M_PI},{11. / 16. * M_PI,7. / 8. * M_PI},{12. / 16. * M_PI,7. / 8. * M_PI},
    {13. / 16. * M_PI,7. / 8. * M_PI},{14. / 16. * M_PI,7. / 8. * M_PI},{15. / 16. * M_PI,7. / 8. * M_PI},{M_PI,7. / 8. * M_PI},
    {9. / 16. * M_PI,8. / 8. * M_PI},{18. / 8. * M_PI / 16. * M_PI,8. / 8. * M_PI},{11. / 16. * M_PI,8. / 8. * M_PI},{12. / 16. * M_PI,8. / 8. * M_PI},
    {13. / 16. * M_PI,8. / 8. * M_PI},{14. / 16. * M_PI,8. / 8. * M_PI},{15. / 16. * M_PI,8. / 8. * M_PI},{M_PI,8. / 8. * M_PI},
    {9. / 16. * M_PI,9. / 8. * M_PI},{19. / 8. * M_PI / 16. * M_PI,9. / 8. * M_PI},{11. / 16. * M_PI,9. / 8. * M_PI},{12. / 16. * M_PI,9. / 8. * M_PI},
    {13. / 16. * M_PI,9. / 8. * M_PI},{14. / 16. * M_PI,9. / 8. * M_PI},{15. / 16. * M_PI,9. / 8. * M_PI},{M_PI,9. / 8. * M_PI},
    {9. / 16. * M_PI,10. / 8. * M_PI},{10. / 16. * M_PI,10. / 8. * M_PI},{11. / 16. * M_PI,10. / 8. * M_PI},{12. / 16. * M_PI,10. / 8. * M_PI},
    {13. / 16. * M_PI,10. / 8. * M_PI},{14. / 16. * M_PI,10. / 8. * M_PI},{15. / 16. * M_PI,10. / 8. * M_PI},{M_PI,10. / 8. * M_PI},
    {9. / 16. * M_PI,11. / 8. * M_PI},{10. / 16. * M_PI,11. / 8. * M_PI},{11. / 16. * M_PI,11. / 8. * M_PI},{12. / 16. * M_PI,11. / 8. * M_PI},
    {13. / 16. * M_PI,11. / 8. * M_PI},{14. / 16. * M_PI,11. / 8. * M_PI},{15. / 16. * M_PI,11. / 8. * M_PI},{M_PI,11. / 8. * M_PI},
    {9. / 16. * M_PI,12. / 8. * M_PI},{10. / 16. * M_PI,12. / 8. * M_PI},{11. / 16. * M_PI,12. / 8. * M_PI},{12. / 16. * M_PI,12. / 8. * M_PI},
    {13. / 16. * M_PI,12. / 8. * M_PI},{14. / 16. * M_PI,12. / 8. * M_PI},{15. / 16. * M_PI,12. / 8. * M_PI},{M_PI,12. / 8. * M_PI},
    {9. / 16. * M_PI,13. / 8. * M_PI},{10. / 16. * M_PI,13. / 8. * M_PI},{11. / 16. * M_PI,13. / 8. * M_PI},{12. / 16. * M_PI,13. / 8. * M_PI},
    {13. / 16. * M_PI,13. / 8. * M_PI},{14. / 16. * M_PI,13. / 8. * M_PI},{15. / 16. * M_PI,13. / 8. * M_PI},{M_PI,13. / 8. * M_PI},
    {9. / 16. * M_PI,14. / 8. * M_PI},{10. / 16. * M_PI,14. / 8. * M_PI},{11. / 16. * M_PI,14. / 8. * M_PI},{12. / 16. * M_PI,14. / 8. * M_PI},
    {13. / 16. * M_PI,14. / 8. * M_PI},{14. / 16. * M_PI,14. / 8. * M_PI},{15. / 16. * M_PI,14. / 8. * M_PI},{M_PI,14. / 8. * M_PI},
    {9. / 16. * M_PI,15. / 8. * M_PI},{10. / 16. * M_PI,15. / 8. * M_PI},{11. / 16. * M_PI,15. / 8. * M_PI},{12. / 16. * M_PI,15. / 8. * M_PI},
    {13. / 16. * M_PI,15. / 8. * M_PI},{14. / 16. * M_PI,15. / 8. * M_PI},{15. / 16. * M_PI,15. / 8. * M_PI},{M_PI,15. / 8. * M_PI},
   }
};

std::tuple<Eigen::Vector3d,Eigen::Vector3d, ompStatus>
nelderMeadMethod(
    const Eigen::Vector3d& ori, const double& orirec,
    const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans,
    const std::vector<std::unordered_set<int>>& watchingVs,
    const Eigen::Matrix3d& v_intrinsic_matrix,
    const double v_dmax, const double v_fov_degree,
    const RTCScene& v_embree_scene
)
{
    srand(time(nullptr));
    Eigen::Vector3d
        vpp1 = ori,
        vpp2 = ori,
        vpp3 = ori,
        vpp4 = ori;
    vpp2.x() = vpp2.x() * (1. + rand() % 8 * 0.01);
    vpp3.y() = vpp3.y() * (1. + rand() % 8 * 0.01);
    vpp4.z() = vpp4.z() * (1. + rand() % 8 * 0.01);
    
    std::array<std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>, 4> vpps{
        {
            {vpp1,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp2,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp3,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp4,0.,Eigen::Vector3d(0.,0.,0.)}
        }
    };

    int itnum = 10;
    ompStatus stts = init;
    while (itnum > 0) {
        itnum--;
        for (int i = 0; i < 4; ++i)
        {
            const auto [maxrec, dirctn] =
                maxREC(
                    get<0>(vpps[i]), v_intrinsic_matrix, v_embree_scene,
                    watchingVs, v_dmax,v_fov_degree, pts, mesh, ans
                );
            get<1>(vpps[i]) = maxrec;
            get<2>(vpps[i]) = dirctn;
        }

        if (get<1>(vpps[3]) - get<1>(vpps[2]) < 1e-3 &&
            get<1>(vpps[3]) - get<1>(vpps[1]) < 1e-3 &&
            get<1>(vpps[3]) - get<1>(vpps[0]) < 1e-3 &&
            get<1>(vpps[3]) == 0.)
        {
            stts = failure;
            break;
        }

        std::sort(
            vpps.begin(), vpps.end(),
            [](const std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>& a, const std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>& b)
            {
                return get<1>(a) < get<1>(b);
            }
        );

        const double diff = std::abs(get<1>(vpps[3]) - get<1>(vpps[0]));
        //Eigen::Vector3d dis = get<0>(vpps[3]) - get<0>(vpps[0]);
        //if (dis.squaredNorm() < 0.1)
        //if (diff < std::abs(get<1>(vpps[3]) * 0.05) || get<1>(vpps[0]) < -3000)
        if (diff < std::abs(get<1>(vpps[3]) * 0.05))
    	{
            stts = slow;
            break;
        }
        if (std::abs(get<1>(vpps[0])) > orirec)
        {
            stts = good;
            break;
        }
        
        Eigen::Vector3d m = (1. / 3.) * (get<0>(vpps.at(0)) + get<0>(vpps.at(1)) + get<0>(vpps.at(2)));
        Eigen::Vector3d r = 2. * m - get<0>(vpps.at(3));
        const auto [rrec, rdirctn] =
            maxREC(
                r, v_intrinsic_matrix, v_embree_scene,
                watchingVs, v_dmax,v_fov_degree, pts, mesh, ans
            );

        if (get<1>(vpps.at(0)) <= rrec && rrec < get<1>(vpps.at(2)))
        {
            get<0>(vpps[3]) = r;
            get<1>(vpps[3]) = rrec;
            get<2>(vpps[3]) = rdirctn;
        }
        else if (get<1>(vpps.at(0)) > rrec)
        {
            Eigen::Vector3d s = m + 2. * (m - get<0>(vpps[3]));
            const auto [srec, sdirctn] =
                maxREC(
                    s, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, v_dmax,v_fov_degree, pts, mesh, ans
                );
            if (rrec > srec)
            {
                get<0>(vpps[3]) = s;
                get<1>(vpps[3]) = srec;
                get<2>(vpps[3]) = sdirctn;
            }
            else
            {
                get<0>(vpps[3]) = r;
                get<1>(vpps[3]) = rrec;
                get<2>(vpps[3]) = rdirctn;
            }
        }
        else if (rrec >= get<1>(vpps.at(2)) && rrec < get<1>(vpps.at(3)))
        {
            Eigen::Vector3d c1 = m + (r - m) / 2.;
            const auto [c1rec, c1dirctn] =
                maxREC(
                    c1, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, v_dmax,v_fov_degree, pts, mesh, ans
                );
            if (c1rec < rrec)
            {
                get<0>(vpps[3]) = c1;
                get<1>(vpps[3]) = c1rec;
                get<2>(vpps[3]) = c1dirctn;
            }
            else
            {
                for (int i = 1; i < 4; i++)
                {
                    get<0>(vpps[i]) = get<0>(vpps[0]) + (get<0>(vpps[i]) - get<0>(vpps[0])) / 2.;
                }
            }
        }
        else // rrec >= get<1>(vpps.at(3))
        {
            Eigen::Vector3d c2 = m + (get<0>(vpps[3]) - m) / 2.;
            const auto [c2rec, c2dirctn] =
                maxREC(
                    c2, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, v_dmax, v_fov_degree, pts, mesh, ans
                );
            if (c2rec < get<1>(vpps[3]))
            {
                get<0>(vpps[3]) = c2;
                get<1>(vpps[3]) = c2rec;
                get<2>(vpps[3]) = c2dirctn;
            }
            else
            {
                for (int i = 1; i < 4; i++)
                {
                    get<0>(vpps[i]) = get<0>(vpps[0]) + (get<0>(vpps[i]) - get<0>(vpps[0])) / 2.;
                }
            }
        }
    }
    if (itnum == 0)
        stts = unenough;
	
    return { get<0>(vpps[0]), get<2>(vpps[0]), stts };
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, ompStatus> nelderMeadMethodPREC(
    const Eigen::Vector3d& ori, const double& orirec, const std::vector<double>& recpts,
    const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans,
    const std::vector<std::unordered_set<int>>& watchingVs,
    const Eigen::Matrix3d& v_intrinsic_matrix,
    const double v_dmax, const double v_fov_degree,
    const RTCScene& v_embree_scene, const modeltools::Height_map& heightMap
)
{
    srand(time(nullptr));
    Eigen::Vector3d
        vpp1 = ori,
        vpp2 = ori,
        vpp3 = ori,
        vpp4 = ori;

    vpp2.x() = vpp2.x() + (-steplength + 1 + rand() % static_cast<int>(steplength) * 2);
    vpp3.y() = vpp3.y() + (-steplength + 1 + rand() % static_cast<int>(steplength) * 2);
    vpp4.z() = vpp4.z() + (-steplength + 1 + rand() % static_cast<int>(steplength) * 2);

    std::array<std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>, 4> vpps{
        {
            {vpp1,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp2,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp3,0.,Eigen::Vector3d(0.,0.,0.)},
            {vpp4,0.,Eigen::Vector3d(0.,0.,0.)}
        }
    };

    int itnum = 3;
    ompStatus stts = init;
    //std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();

    while (itnum > 0) {
        itnum--;
        for (int i = 0; i < 4; ++i)
        {
            const auto [maxrec, dirctn] =
                maxUTL(
                    get<0>(vpps[i]), v_intrinsic_matrix, v_embree_scene,
                    watchingVs, recpts, v_dmax, v_fov_degree, pts, mesh, ans
                );
            get<1>(vpps[i]) = maxrec;
            get<2>(vpps[i]) = dirctn;
        }
        //checkpointTime(t, std::to_string(itnum) + "fisrt max");
        std::sort(
            vpps.begin(), vpps.end(),
            [](const std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>& a, const std::tuple<Eigen::Vector3d, double, Eigen::Vector3d>& b)
            {
                return get<1>(a) < get<1>(b);
            }
        );
        //checkpointTime(t, std::to_string(itnum) + "sort");
        const double diff = std::abs(get<1>(vpps[3]) - get<1>(vpps[0]));

        if (std::abs(get<1>(vpps[0])) > orirec)
        {
            stts = good;
            break;
        }

        if (diff < std::abs(get<1>(vpps[3]) * 0.2))
        {
            stts = slow;
            break;
        }

        Eigen::Vector3d m = (1. / 3.) * (get<0>(vpps.at(0)) + get<0>(vpps.at(1)) + get<0>(vpps.at(2)));
        Eigen::Vector3d r = 2. * m - get<0>(vpps.at(3));
        const auto [rrec, rdirctn] =
            maxUTL(
                r, v_intrinsic_matrix, v_embree_scene,
                watchingVs, recpts, v_dmax, v_fov_degree, pts, mesh, ans
            );

        if (get<1>(vpps.at(0)) <= rrec && rrec < get<1>(vpps.at(2)))
        {
            get<0>(vpps[3]) = r;
            get<1>(vpps[3]) = rrec;
            get<2>(vpps[3]) = rdirctn;
        }
        else if (get<1>(vpps.at(0)) > rrec)
        {
            Eigen::Vector3d s = m + 2. * (m - get<0>(vpps[3]));
            const auto [srec, sdirctn] =
                maxUTL(
                    s, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, recpts, v_dmax, v_fov_degree, pts, mesh, ans
                );
            if (rrec > srec)
            {
                get<0>(vpps[3]) = s;
                get<1>(vpps[3]) = srec;
                get<2>(vpps[3]) = sdirctn;
            }
            else
            {
                get<0>(vpps[3]) = r;
                get<1>(vpps[3]) = rrec;
                get<2>(vpps[3]) = rdirctn;
            }
        }
        else if (rrec >= get<1>(vpps.at(2)) && rrec < get<1>(vpps.at(3)))
        {
            Eigen::Vector3d c1 = m + (r - m) / 2.;
            const auto [c1rec, c1dirctn] =
                maxUTL(
                    c1, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, recpts, v_dmax, v_fov_degree, pts, mesh, ans
                );
            if (c1rec < rrec)
            {
                get<0>(vpps[3]) = c1;
                get<1>(vpps[3]) = c1rec;
                get<2>(vpps[3]) = c1dirctn;
            }
            else
            {
                for (int i = 1; i < 4; i++)
                {
                    get<0>(vpps[i]) = get<0>(vpps[0]) + (get<0>(vpps[i]) - get<0>(vpps[0])) / 2.;
                }
            }
        }
        else // rrec >= get<1>(vpps.at(3))
        {
            Eigen::Vector3d c2 = m + (get<0>(vpps[3]) - m) / 2.;
            const auto [c2rec, c2dirctn] =
                maxUTL(
                    c2, v_intrinsic_matrix, v_embree_scene,
                    watchingVs, recpts, v_dmax, v_fov_degree, pts, mesh, ans
                );
            if (c2rec < get<1>(vpps[3]))
            {
                get<0>(vpps[3]) = c2;
                get<1>(vpps[3]) = c2rec;
                get<2>(vpps[3]) = c2dirctn;
            }
            else
            {
                for (int i = 1; i < 4; i++)
                {
                    get<0>(vpps[i]) = get<0>(vpps[0]) + (get<0>(vpps[i]) - get<0>(vpps[0])) / 2.;
                }
            }
        }
        //checkpointTime(t, std::to_string(itnum) + "ad");
    }// while

    if (itnum == 0)
        stts = unenough;

    if (!heightMap.is_safe(get<0>(vpps[0])) || get<0>(vpps[0]).z() < 10.0)
    {
        stts = failure;
    }
    //LOG(INFO) << itnum << "\t" << stts;
    return { get<0>(vpps[0]), get<2>(vpps[0]), stts };
}

std::pair<double, Eigen::Vector3d> maxREC(
    const Eigen::Vector3d& postion,
    const Eigen::Matrix3d& v_intrinsic_matrix,
    const RTCScene& v_embree_scene,
    const std::vector<std::unordered_set<int>>& watchingVs,
    const double v_dmax, const double v_fov_degree,
    const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans
)
{
    std::array<double, 128> rec;
    rec.fill(0.);

#pragma omp parallel for
    for (int i = 0; i < 128; ++i)
    {
        Eigen::Vector3d dirctn(
            std::sin(viewCandidates[i].first) * std::cos(viewCandidates[i].second), 
            std::sin(viewCandidates[i].first) * std::sin(viewCandidates[i].second),
            std::cos(viewCandidates[i].first));
        dirctn = dirctn / std::sqrt(dirctn.squaredNorm());
        double currec = 0.;

        for (int j = 0; j < pts.size(); ++j)
        {
            if (intersectiontools::is_visible(postion, dirctn, v_fov_degree,
                                              cgaltools::cgal_point_2_eigen(pts.point(j)), v_embree_scene, v_dmax, mesh)
                )
            {
                for (const auto& k : watchingVs.at(j))
                {
                    const double v = biviewsREC(
                        postion, ans.at(k).pos_mesh,
                        cgaltools::cgal_point_2_eigen(pts.point(j)), cgaltools::cgal_vector_2_eigen(pts.normal(j)), v_dmax);
                        currec += v;
                }
            }
        }
        rec[i] = currec;
    }
    auto maxitr = std::max_element(rec.begin(), rec.end());
    const int index = maxitr - rec.begin();
    Eigen::Vector3d maxdirctn(
        std::sin(viewCandidates[index].first) * std::cos(viewCandidates[index].second),
        std::sin(viewCandidates[index].first) * std::sin(viewCandidates[index].second),
        std::cos(viewCandidates[index].first));
    maxdirctn /= std::sqrt(maxdirctn.squaredNorm());

    return { -*maxitr, maxdirctn };
}

std::pair<double, Eigen::Vector3d> maxUTL(
    const Eigen::Vector3d& postion,
    const Eigen::Matrix3d& v_intrinsic_matrix,
    const RTCScene& v_embree_scene,
    const std::vector<std::unordered_set<int>>& watchingVs, const std::vector<double> recpts,
    const double v_dmax, const double v_fov_degree,
    const PointSet3& pts, const SurfaceMesh& mesh, const std::vector<Viewpoint>& ans
)
{
    std::array<double, 128> score;
    score.fill(0.);


    for (int i = 0; i < 128; ++i)
    {
        Eigen::Vector3d dirctn(
            std::sin(viewCandidates[i].first) * std::cos(viewCandidates[i].second),
            std::sin(viewCandidates[i].first) * std::sin(viewCandidates[i].second),
            std::cos(viewCandidates[i].first));
        dirctn = dirctn / std::sqrt(dirctn.squaredNorm());
        double curscr = 0.;

#pragma omp parallel for
        for (int j = 0; j < pts.size(); ++j)
        {
            if (recpts[j] < hallmark) {
                if (intersectiontools::is_visible(postion, dirctn, v_fov_degree,
                                                  cgaltools::cgal_point_2_eigen(pts.point(j)), v_embree_scene, v_dmax, mesh)
                    )
                {
                    for (const auto& k : watchingVs.at(j))
                    {
#pragma omp critical
                        {
                            curscr += biviewsREC(
                                postion, ans.at(k).pos_mesh,
                                cgaltools::cgal_point_2_eigen(pts.point(j)), cgaltools::cgal_vector_2_eigen(pts.normal(j)), v_dmax);
                        }
                    }
                }
            }
        }
        score[i] = curscr;
    }

    const auto maxitr = std::max_element(score.begin(), score.end());
    const int index = maxitr - score.begin();
    Eigen::Vector3d maxdirctn(
        std::sin(viewCandidates[index].first) * std::cos(viewCandidates[index].second),
        std::sin(viewCandidates[index].first) * std::sin(viewCandidates[index].second),
        std::cos(viewCandidates[index].first));
    maxdirctn /= std::sqrt(maxdirctn.squaredNorm());

    return { -*maxitr, maxdirctn };
}


std::vector<Viewpoint> adjustTraj(
    const std::vector<Viewpoint>& traj, const modeltools::Height_map& heightMap,
    const PointSet3& pts, const SurfaceMesh& mesh,
    const Eigen::Matrix3d& v_intrinsic_matrix,
    const double v_dmax, const double v_fov_degree,
    SmithViz* v_viz
)
{
    auto embree_scene = intersectiontools::generate_embree_scene(mesh);
    const std::vector<std::vector<int>> visibility = compute_visibilityIndex(
        traj, mesh, pts, v_intrinsic_matrix, v_dmax, embree_scene
    );
    const int vSize = static_cast<int>(traj.size());
    const int pSize = static_cast<int>(pts.size());
    LOG(INFO) << "Vsize: " << vSize << "  Psize: " << pSize;
    
    std::vector<Viewpoint> ans = traj;

    std::vector<Eigen::Vector3d> ptsp(pSize), ptsn(pSize);
    for (int i = 0; i < pSize; i++)
    {
        ptsp.at(i) = cgaltools::cgal_point_2_eigen(pts.point(i));
        ptsn.at(i) = cgaltools::cgal_vector_2_eigen(pts.normal(i));
    }
    std::vector<std::vector<int>> vsss = compute_visibilityIndex(ans, mesh, pts, v_intrinsic_matrix, v_dmax, embree_scene);
   
    auto curREC = totalRECv(ans, ptsp, ptsn, vsss);
    LOG(INFO) << "Initial REC: " << std::reduce(curREC.begin(), curREC.end()) / pSize;


    // ****************************************************************************
    std::ofstream file1("C:/Marinaio/TEMP/sig18test1.txt");
    std::ofstream file2("C:/Marinaio/TEMP/sig18test2.txt");
    // ****************************************************************************

//    std::vector<double> orirecOfv(vSize);
//#pragma omp parallel for
//    for (int i = 0; i < vSize; ++i)
//    {
//        double currec = 0.;
//        for (const auto& j : monitoredPts.at(i))
//        {
//            for (const auto& k : watchingVs.at(j))
//            {
//                if (i != k)
//					currec += biviewsREC(
//                        traj.at(i).pos_mesh, traj.at(k).pos_mesh, 
//                        cgal_point_2_eigen(pts.point(j)), cgal_vector_2_eigen(pts.normal(j)), v_dmax
//                    );
//            }
//        }
//#pragma omp critical
//        {
//            // ****************************************************************************
//            //file1 << traj.at(i).pos_mesh.transpose() << ": \t" << currec << "\n";
//            // ****************************************************************************
//            orirecOfv.at(i) = currec;
//        }
//    }
//    LOG(INFO) << "Initial recs of views are computed.";

//    std::vector<double> recpts(pSize);
//#pragma omp parallel for
//    for (int i = 0; i < pSize; ++i)
//    {
//        double currec = 0.;
//        for (int j = 0; j < watchingVs.at(i).size(); ++j)
//        {
//            for (int k = j + 1; k < watchingVs.at(i).size(); ++k)
//            {
//                currec += biviewsREC(
//                    traj.at(j).pos_mesh, traj.at(k).pos_mesh,
//                    cgal_point_2_eigen(pts.point(i)), cgal_vector_2_eigen(pts.normal(i)),
//                    v_dmax
//                );
//            }
//        }
//        recpts[i] = currec;
//    }
//    LOG(INFO) << "Initial recs of pts are computed.";

    int OIN = outerIternum;
    while (OIN > 0) {
        OIN--;

        PointSet3 toBA;
        toBA.add_normal_map();
        std::vector<double> underBaseline;
        for (int i = 0; i < pSize; i++)
        {
            if (curREC.at(i) < upperHallmark)
            {
                toBA.insert(pts.point(i), pts.normal(i));
                underBaseline.push_back(curREC.at(i));
            }
        }

        const int tbaSize = static_cast<int>(toBA.size());
        std::vector<std::unordered_set<int>> watchingVs(tbaSize);
        std::vector<std::unordered_set<int>> monitoredPts(vSize);

        auto toBAvis = compute_visibilityIndex(ans, mesh, toBA, v_intrinsic_matrix, v_dmax, embree_scene);

        LOG(INFO) << "TOBA are initialized. It's size is: " << toBA.size();

#pragma omp parallel for
        for (int i = 0; i < tbaSize; ++i)
        {
            auto vis = toBAvis[i];
            for (int j = 0; j < vis.size(); ++j)
            {
#pragma omp critical
                {
                    monitoredPts.at(vis[j]).insert(i);
                    watchingVs.at(i).insert(vis[j]);
                }
            }
        }
        LOG(INFO) << "monitoredpts and watchingvs are initialized.";

        std::vector<double> oriscrOfv(vSize);

#pragma omp parallel for
        for (int i = 0; i < vSize; ++i) {
            double curscr = 0.;
           
            for (const auto& j : monitoredPts.at(i))
            {
                for (const auto& k : watchingVs.at(j))
                {
                    if (i != k) {
                        const double v = biviewsREC(
                            traj.at(i).pos_mesh, traj.at(k).pos_mesh,
                            cgaltools::cgal_point_2_eigen(pts.point(j)), cgaltools::cgal_vector_2_eigen(pts.normal(j)), v_dmax);
                        curscr += v;
                    }
                }
            }
            oriscrOfv[i] = curscr;
        }
        LOG(INFO) << "Initial scores of views are computed.";

        int gd = 0;
        int bd = 0;

        int IIN = innerItrnum;
        while (IIN > 0)
        {
            IIN -= 1;

            std::vector<std::pair<int, ompStatus>> toBeAdjusted;
            srand(time(nullptr));

            std::vector<int> aView(vSize, 0);
            int curIndex = rand() % vSize;
            //toBeAdjusted.size() <= 10 &&
            int whileNum = 0;
            while (std::reduce(aView.begin(), aView.end()) != vSize)
            {
                if (!aView[curIndex]) {

                    toBeAdjusted.push_back({ curIndex, init });
                    aView.at(curIndex) = 1;
                    
                    for (const auto& i : monitoredPts.at(curIndex))
                    {
                        Eigen::Vector3d ppos = cgaltools::cgal_point_2_eigen(toBA.point(i));
                        Eigen::Vector3d pnom = cgaltools::cgal_vector_2_eigen(toBA.normal(i));

                        watchingVs.at(i).erase(watchingVs.at(i).find(curIndex));
                        for (const auto& j : watchingVs.at(i))
                        {
                            if (curIndex != j) {
                                underBaseline.at(i) -= biviewsREC(
                                    ans.at(curIndex).pos_mesh, ans.at(j).pos_mesh,
                                    ppos, pnom, v_dmax
                                );
                            }
                            aView[j] = 1;
                        }
                    }
                }
                curIndex++;
                if (curIndex == vSize)
                {
                    curIndex = 0;
                }
                whileNum++;
                if (whileNum > vSize)
                    break;
            }

            std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
            //if ((1 + IIN) % 25 == 0) {
                LOG(INFO) << toBeAdjusted.size() << " views will be adjusted.";
            //}

            for (int i = 0; i < toBeAdjusted.size(); ++i)
            {
                Viewpoint& vp = ans[toBeAdjusted[i].first];
                const auto [pos, dirctn, stts] =
                    nelderMeadMethodPREC(
                        vp.pos_mesh, oriscrOfv.at(toBeAdjusted[i].first), underBaseline,
                        toBA, mesh, ans, watchingVs, v_intrinsic_matrix, v_dmax, v_fov_degree,
                        embree_scene, heightMap
                    );
                toBeAdjusted[i].second = stts;
               // if ((IIN + 1) % 25 == 0) {
                  //  checkpointTime(t, "NELDERMEADMETHOD: ");
                //}
                if (stts == good) {
                    //LOG(INFO) << "good.";
                    gd++;
                	vp.pos_mesh = pos;
                    vp.direction = dirctn;

                    oriscrOfv.at(toBeAdjusted[i].first) = 0.;
                    monitoredPts.at(toBeAdjusted[i].first).clear();
#pragma omp parallel for
                    for (int j = 0; j < tbaSize; ++j)
                    {
                        if (intersectiontools::is_visible(pos, dirctn, v_fov_degree,
                                                          cgaltools::cgal_point_2_eigen(toBA.point(j)), embree_scene, v_dmax, mesh)
                            )
                        {
#pragma omp critical
                            {
                                monitoredPts.at(toBeAdjusted[i].first).insert(j);
                                
                                Eigen::Vector3d ppos = cgaltools::cgal_point_2_eigen(toBA.point(j));
                                Eigen::Vector3d pnom = cgaltools::cgal_vector_2_eigen(toBA.normal(j));
                                for (const auto& k : watchingVs.at(j))
                                {
                                    underBaseline.at(j) += biviewsREC(
                                        pos, ans.at(k).pos_mesh,
                                        ppos, pnom, v_dmax
                                    );
                                }

                                watchingVs.at(j).insert(toBeAdjusted[i].first);
                            }
                        }
                    }
                }
                else
                {
                    bd++;
                    for (const auto& j : monitoredPts.at(toBeAdjusted[i].first))
                    {
                        Eigen::Vector3d ppos = cgaltools::cgal_point_2_eigen(toBA.point(j));
                        Eigen::Vector3d pnom = cgaltools::cgal_vector_2_eigen(toBA.normal(j));
                        for (const auto& k : watchingVs.at(j))
                        {
                            underBaseline.at(j) += biviewsREC(
                                vp.pos_mesh, ans.at(k).pos_mesh,
                                ppos, pnom, v_dmax
                            );
                        }
                        watchingVs.at(j).insert(toBeAdjusted[i].first);
                    }
                }
                
            }
          
#pragma omp parallel for
            for (int i = 0; i < toBeAdjusted.size(); ++i)
            {
                if (toBeAdjusted[i].second == good) {
                    double curscr = 0.;

                    for (const auto& j : monitoredPts.at(toBeAdjusted[i].first))
                    {
                        for (const auto& k : watchingVs.at(j))
                        {
                            if (toBeAdjusted[i].first != k) {
                                curscr += biviewsREC(
                                    traj.at(toBeAdjusted[i].first).pos_mesh, traj.at(k).pos_mesh,
                                    cgaltools::cgal_point_2_eigen(toBA.point(j)), cgaltools::cgal_vector_2_eigen(toBA.normal(j)), v_dmax);
                            }
                        }
                    }
                    oriscrOfv[toBeAdjusted[i].first] = curscr;
                }
            }
        }// II
        LOG(INFO) << "GOOD: " << gd << "\tBAD: " << bd;
        vsss = compute_visibilityIndex(ans, mesh, pts, v_intrinsic_matrix, v_dmax, embree_scene);
        curREC = totalRECv(ans, ptsp, ptsn, vsss);
    }//OI

    write_normal_path(ans, fs::path("log/smithans.txt").string());
    LOG(INFO) << "start to computecurREC.";
    vsss = compute_visibilityIndex(ans, mesh, pts, v_intrinsic_matrix, v_dmax, embree_scene);
    auto totalCURREC = totalREC(ans, ptsp, ptsn, vsss);
    LOG(INFO) << "curREC: " << totalCURREC / pSize;
    
    return ans;
}
*/

/*
 * New interface.
 */

int candidateViewsDens = 256;

std::vector<Viewpoint> SmithAdj(
    const std::vector<Viewpoint>& views, const PointSet3& pts,
    const SurfaceMesh& mesh,
    const Eigen::Matrix3d& intrinsicMatrix, const double viewDis, const double fov,
    const int& CVD
)
{
    initialization(mesh);

    std::vector<Viewpoint> temp;
    return temp;
}

std::vector<Viewpoint> adjusting(
    const std::vector<Viewpoint>& views, const PointSet3& pts,
    const SurfaceMesh& mesh,
    const Eigen::Matrix3d& intrinsicMatrix, const double viewDis, const double fov
)
{
    cudaFree(0);
	optixInit();
    OptixDeviceContextOptions options = {};
    CUcontext cuCtx = 0;
    OptixDeviceContext context = nullptr;
    optixDeviceContextCreate(cuCtx, &options, &context);

    // TODO: it's wrong now.
    return views;
}

void initialization(const SurfaceMesh& mesh, const int& CVD)
{
    if (CVD != 256)
    {
        candidateViewsDens = CVD;
    }
    
    const int verStripAmount = static_cast<int>(std::ceil((2 + std::sqrt(4 - 8 * (-candidateViewsDens))) / 4));
    const int horiStripAmount = 2 * verStripAmount - 2;
    verStripAngle = M_PI / (verStripAmount - 1);
    horiStripAngle = (2 * M_PI) / horiStripAmount;
    candidateViewsDens = verStripAmount * horiStripAmount;
    double2* viewsOffsetCu;
    cudaMalloc(&viewsOffsetCu, candidateViewsDens * sizeof(double2));

    int blockNum = candidateViewsDens / tpb + 1;
    initOffsetInf(blockNum, tpb, viewsOffsetCu, candidateViewsDens, horiStripAmount, verStripAngle, horiStripAngle);
    conveyFromCuda(viewsOffsetCu, viewsOffsetC, candidateViewsDens);
    cudaFree(viewsOffsetCu);

}