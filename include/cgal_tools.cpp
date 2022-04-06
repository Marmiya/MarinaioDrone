#include "cgal_tools.h"

namespace cgaltools {

    SurfaceMesh convert_obj_from_tinyobjloader_to_surface_mesh(
        const std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>> v_obj_in) {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::tie(attrib, shapes, materials) = v_obj_in;

        SurfaceMesh out;

        std::unordered_map<int, SurfaceMesh::vertex_index> vertex_map;

        std::vector<Point3> points;
        std::vector<std::vector<size_t>> polygon;

        for (size_t s = 0; s < shapes.size(); s++) {
            size_t index_offset = 0;
            for (size_t face_id = 0; face_id < shapes[s].mesh.num_face_vertices.size(); face_id++) {
                if (shapes[s].mesh.num_face_vertices[face_id] != 3)
                    throw;

                SurfaceMesh::vertex_index vi[3];
                for (size_t v = 0; v < 3; v++) {
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                    tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                    tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                    tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
                    points.push_back(Point3(vx, vy, vz));
                    if (vertex_map.find(idx.vertex_index) == vertex_map.end())
                    {
                        vertex_map.insert(std::make_pair(idx.vertex_index, out.add_vertex(Point3(vx, vy, vz))));
                        vi[v] = vertex_map.at(idx.vertex_index);
                    }
                    else
                        vi[v] = vertex_map.at(idx.vertex_index);
                }
                out.add_face(vi[0], vi[1], vi[2]);
                polygon.push_back(std::vector<size_t>{points.size() - 2, points.size() - 1, points.size()});
                index_offset += 3;
            }
        }
        SurfaceMesh out1;

        return out;
    }

    Eigen::AlignedBox3d get_bounding_box(const PointSet3& v_point_set)
    {
        float xmin = 1e8, ymin = 1e8, zmin = 1e8;
        float xmax = -1e8, ymax = -1e8, zmax = -1e8;

        for (PointSet3::Index idx : v_point_set)
        {
            float vx = v_point_set.point(idx).x();
            float vy = v_point_set.point(idx).y();
            float vz = v_point_set.point(idx).z();
            xmin = xmin < vx ? xmin : vx;
            ymin = ymin < vy ? ymin : vy;
            zmin = zmin < vz ? zmin : vz;

            xmax = xmax > vx ? xmax : vx;
            ymax = ymax > vy ? ymax : vy;
            zmax = zmax > vz ? zmax : vz;
        }


        return Eigen::AlignedBox3d(Eigen::Vector3d(xmin, ymin, zmin), Eigen::Vector3d(xmax, ymax, zmax));
    }

    RotatedBox get_bounding_box_rotated(const PointSet3& v_point_set)
    {
        float xmin = 1e8, ymin = 1e8, zmin = 1e8;
        float xmax = -1e8, ymax = -1e8, zmax = -1e8;

        std::vector<cv::Point2f> points;

        for (PointSet3::Index idx : v_point_set)
        {
            float vx = v_point_set.point(idx).x();
            float vy = v_point_set.point(idx).y();
            float vz = v_point_set.point(idx).z();
            xmin = xmin < vx ? xmin : vx;
            ymin = ymin < vy ? ymin : vy;
            zmin = zmin < vz ? zmin : vz;

            xmax = xmax > vx ? xmax : vx;
            ymax = ymax > vy ? ymax : vy;
            zmax = zmax > vz ? zmax : vz;
            points.emplace_back(vx, vy);
        }

        cv::RotatedRect rotated_rect = cv::minAreaRect(points);

        RotatedBox box(Eigen::AlignedBox3d(
            Eigen::Vector3d(rotated_rect.center.x - rotated_rect.size.width / 2, rotated_rect.center.y - rotated_rect.size.height / 2, zmin),
            Eigen::Vector3d(rotated_rect.center.x + rotated_rect.size.width / 2, rotated_rect.center.y + rotated_rect.size.height / 2, zmax)));

        box.angle = rotated_rect.angle / 180.f * 3.1415926f;
        box.cv_box = rotated_rect;
        return box;
    }

    Eigen::Vector3d cgal_point_2_eigen(const Point3& p)
    {
        return Eigen::Vector3d(p.x(), p.y(), p.z());
    }

    Eigen::Vector3d cgal_vector_2_eigen(const Vector3& p)
    {
        return Eigen::Vector3d(p.x(), p.y(), p.z());
    }

    Point3 eigen_2_cgal_point(const Eigen::Vector3d& p)
    {
        return Point3(p.x(), p.y(), p.z());
    }

    Vector3 eigen_2_cgal_vector(const Eigen::Vector3d& p)
    {
        return Vector3(p.x(), p.y(), p.z());
    }

    Cuboid::Cuboid(std::set<Point3> s)
    {

        std::vector<Point3> downPts, upPts;
        double downZ = s.begin()->z(), upZ = (++s.begin())->z();

        if (upZ < downZ) {
            boost::swap(upZ, downZ);
        }
        else if (upZ == downZ)
        {
            for (const auto& i : s) {
                if (i.z() > upZ) {
                    upZ = i.z();
                    break;
                }
                if (i.z() < upZ) {
                    downZ = i.z();
                    break;
                }
            }
        }

        for (auto& i : s) {
            if (i.z() == upZ) {
                upPts.push_back(i);
            }
            else {
                downPts.push_back(i);
            }
        }

        Point3 fstP = *downPts.cbegin();
        Point2 origin = Point2(CGAL::ORIGIN);
        double dis = CGAL::squared_distance(
            Point2(downPts.begin()->x(), downPts.begin()->x()), origin);
        for (auto i = downPts.cbegin() + 1; i != downPts.cend(); ++i) {
            double curDis = CGAL::squared_distance(Point2(i->x(), i->y()), origin);
            if (curDis < dis) {
                fstP = *i;
            }
        }
        vertices.push_back(fstP);
        downPts.erase(std::find(downPts.begin(), downPts.end(), fstP));
        Point3 trdP;
        dis = .0;
        for (const auto& i : downPts) {
            double curDis = CGAL::squared_distance(i, fstP);
            if (curDis > dis) {
                trdP = i;
            }
        }
        downPts.erase(std::find(downPts.begin(), downPts.end(), trdP));

        Vector3 diag(fstP, trdP);
        Vector3 side(fstP, *downPts.begin());

        auto crossmultiply = CGAL::cross_product<K>(side, diag);
        if (crossmultiply.z() > 0) {
            vertices.push_back(*downPts.begin());
            vertices.push_back(trdP);
            vertices.push_back(*(--downPts.end()));
        }
        else {
            vertices.push_back(*(--downPts.end()));
            vertices.push_back(trdP);
            vertices.push_back(*downPts.begin());
        }

        vertices.push_back(Point3(vertices.at(3).x(), vertices.at(3).y(), upZ));
        vertices.push_back(Point3(vertices.at(0).x(), vertices.at(0).y(), upZ));
        vertices.push_back(Point3(vertices.at(1).x(), vertices.at(1).y(), upZ));
        vertices.push_back(Point3(vertices.at(2).x(), vertices.at(2).y(), upZ));

        {
            segments.push_back(Segment3(vertices.at(0), vertices.at(1)));
            segments.push_back(Segment3(vertices.at(1), vertices.at(2)));
            segments.push_back(Segment3(vertices.at(2), vertices.at(3)));
            segments.push_back(Segment3(vertices.at(3), vertices.at(0)));
            segments.push_back(Segment3(vertices.at(5), vertices.at(6)));
            segments.push_back(Segment3(vertices.at(6), vertices.at(7)));
            segments.push_back(Segment3(vertices.at(7), vertices.at(4)));
            segments.push_back(Segment3(vertices.at(4), vertices.at(5)));
            segments.push_back(Segment3(vertices.at(0), vertices.at(5)));
            segments.push_back(Segment3(vertices.at(1), vertices.at(6)));
            segments.push_back(Segment3(vertices.at(2), vertices.at(7)));
            segments.push_back(Segment3(vertices.at(3), vertices.at(4)));
        }

        // Construct bottomSegs
        bottomSegs = std::make_shared<SegmentLN>(segments.at(0));
        auto snd = std::make_shared<SegmentLN>(segments.at(1));
        auto trd = std::make_shared<SegmentLN>(segments.at(2));
        auto fth = std::make_shared<SegmentLN>(segments.at(3));
        bottomSegs->setNext(snd);
        snd->setNext(trd);
        trd->setNext(fth);
        fth->setNext(bottomSegs);

    }

    inline double area(const Point3& p1, const Point3& p2, const Point3& p3)
    {
        return 0.5 * CGAL::sqrt(CGAL::squared_distance(p1, p2)) * CGAL::sqrt(CGAL::squared_distance(Line3(p1, p2), p3));
    }

    SurfaceMesh averaged(const SurfaceMesh& mesh, double expectedArea)
    {
        SurfaceMesh m = mesh;
        CGAL::Polygon_mesh_processing::triangulate_faces(m);
        
        while (true)
        {
            int faceSize = static_cast<int>(m.number_of_faces());
            std::vector<bool> ifOverBig(faceSize, false);

        	#pragma omp parallel for
            for (int i = 0; i < faceSize; i++)
            {
                CGAL::Vertex_around_face_iterator<SurfaceMesh> vbegin, vend;
                boost::tie(vbegin, vend) = vertices_around_face(m.halfedge(SurfaceMesh::face_index(i)), m);
                Point3 p1 = m.point(*vbegin), p2 = m.point(*(++vbegin)), p3 = m.point(*(++vbegin));
                if (area(p1, p2, p3) > expectedArea)
                {
				#pragma omp critical
                    {
                        ifOverBig[i] = true;
                    }
                }
            }

            std::vector<int> OBvec;
            for (int i = 0; i < faceSize; i++)
            {
                if (ifOverBig.at(i))
	            {
                    OBvec.push_back(i);
	            }
            }

            if (OBvec.empty())
            {
                break;
            }
            else
            {
				#pragma omp parallel for
                for (int i = 0; i < OBvec.size(); i++)
                {
                    SurfaceMesh::face_index curFI(OBvec.at(i));
                    CGAL::Vertex_around_face_iterator<SurfaceMesh> vbegin, vend;
                    boost::tie(vbegin, vend) = vertices_around_face(m.halfedge(curFI), m);
                    SurfaceMesh::vertex_index ep1VI = *vbegin, ep2VI = *(++vbegin), ep3VI = *(++vbegin);
                    Point3 p1 = m.point(ep1VI), p2 = m.point(ep2VI), p3 = m.point(ep3VI);
                    Point3 oddp1 = CGAL::midpoint(p1, p2), oddp2 = CGAL::midpoint(p2, p3), oddp3 = CGAL::midpoint(p3, p1);
	                #pragma omp critical
                    {
                        m.remove_face(curFI);
                        
                        SMVI p1VI = m.add_vertex(oddp1), p2VI = m.add_vertex(oddp2), p3VI = m.add_vertex(oddp3);

                        if (!m.is_border(ep1VI))
                        {
                            ep1VI = m.add_vertex(p1);
                        }
                        if (!m.is_border(ep2VI))
                        {
                            ep2VI = m.add_vertex(p2);
                        }
                        if (!m.is_border(ep3VI))
                        {
                            ep3VI = m.add_vertex(p3);
                        }

                        std::vector<SMVI> vf1{ ep1VI, p1VI, p3VI };
                        std::vector<SMVI> vf2{ p1VI, ep2VI, p2VI };
                        std::vector<SMVI> vf3{ p1VI, p2VI, p3VI };
                        std::vector<SMVI> vf4{ ep3VI, p3VI, p2VI };

                        SMFI f1 = m.add_face(vf1);
                        SMFI f2 = m.add_face(vf2);
                        SMFI f3 = m.add_face(vf3);
                        SMFI f4 = m.add_face(vf4);
                        if (f4 == SurfaceMesh::null_face() || f3 == SurfaceMesh::null_face() ||
                            f2 == SurfaceMesh::null_face() || f1 == SurfaceMesh::null_face()
                            )
                        {
                            throw;
                        }

                    }
                }
                m.collect_garbage();
            }
        }

        return m;
    }

    SurfaceMesh
	obb(
        const SurfaceMesh& sm, const double& bbExpandBias
    )
    {
        SurfaceMesh obb;
        std::array<Point3, 8> obbpts;
        CGAL::oriented_bounding_box(sm, obbpts);
        if (bbExpandBias != 0.)
        {
            Vector3 xdirection = obbpts.at(1) - obbpts.at(0);
            Vector3 ydirection = obbpts.at(3) - obbpts.at(0);
            Vector3 zdirection = obbpts.at(5) - obbpts.at(0);
            xdirection /= CGAL::sqrt(xdirection.squared_length());
            ydirection /= CGAL::sqrt(ydirection.squared_length());
            zdirection /= CGAL::sqrt(zdirection.squared_length());

            obbpts.at(0) = obbpts.at(0) + bbExpandBias * (-xdirection + (-ydirection));
            obbpts.at(1) = obbpts.at(1) + bbExpandBias * (xdirection + (-ydirection));
            obbpts.at(2) = obbpts.at(2) + bbExpandBias * (xdirection + ydirection);
            obbpts.at(3) = obbpts.at(3) + bbExpandBias * (-xdirection + ydirection);
            obbpts.at(4) = obbpts.at(4) + bbExpandBias * (-xdirection + ydirection + zdirection);
            obbpts.at(5) = obbpts.at(5) + bbExpandBias * (-xdirection + (-ydirection) + zdirection);
            obbpts.at(6) = obbpts.at(6) + bbExpandBias * (xdirection + (-ydirection) + zdirection);
            obbpts.at(7) = obbpts.at(7) + bbExpandBias * (xdirection + ydirection + zdirection);
        }

        CGAL::make_hexahedron(
            obbpts.at(0),
            obbpts.at(1),
            obbpts.at(2),
            obbpts.at(3),
            obbpts.at(4),
            obbpts.at(5),
            obbpts.at(6),
            obbpts.at(7),
            obb
        );
        return obb;
    }

    double squaredDistanceCuboid(Cuboid& l, Cuboid& r)
    {
        auto lv = l.getVer();
        auto rs = r.getSeg();
        std::vector<Point3> lBottomPoints(lv.begin(), lv.begin() + 4);
        std::vector<Segment3> rBottomSegments(rs.begin(), rs.begin() + 4);
        double minSquaredDis = 10000000.0;

        for (const auto& i : lBottomPoints) {
            for (const auto& j : rBottomSegments) {
                double dis = CGAL::squared_distance(i, j);
                if (dis < minSquaredDis)
                    minSquaredDis = dis;
            }
        }

        return minSquaredDis;
    }

    void pointVisualization(
        std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>>& obj,
        Point3 pt, double radius)
    {
        auto& [attrib, shapes, materials] = obj;
        double px = pt.x(), py = pt.y(), pz = pt.z();
        size_t originalSize = attrib.vertices.size() / 3;
        // ADD points
        {
            double underSideFstPx = px - radius;
            double underSideFstPy = py - radius;
            double underSideFstPz = pz - radius;

            double underSideSndPx = px + radius;
            double underSideSndPy = py - radius;
            double underSideSndPz = pz - radius;

            double underSideTrdPx = px + radius;
            double underSideTrdPy = py + radius;
            double underSideTrdPz = pz - radius;

            double underSideFthPx = px - radius;
            double underSideFthPy = py + radius;
            double underSideFthPz = pz - radius;

            double upSideFstPx = px - radius;
            double upSideFstPy = py - radius;
            double upSideFstPz = pz + radius;

            double upSideSndPx = px + radius;
            double upSideSndPy = py - radius;
            double upSideSndPz = pz + radius;

            double upSideTrdPx = px + radius;
            double upSideTrdPy = py + radius;
            double upSideTrdPz = pz + radius;

            double upSideFthPx = px - radius;
            double upSideFthPy = py + radius;
            double upSideFthPz = pz + radius;

            attrib.vertices.push_back(underSideFstPx);
            attrib.vertices.push_back(underSideFstPy);
            attrib.vertices.push_back(underSideFstPz);
            attrib.vertices.push_back(underSideSndPx);
            attrib.vertices.push_back(underSideSndPy);
            attrib.vertices.push_back(underSideSndPz);
            attrib.vertices.push_back(underSideTrdPx);
            attrib.vertices.push_back(underSideTrdPy);
            attrib.vertices.push_back(underSideTrdPz);
            attrib.vertices.push_back(underSideFthPx);
            attrib.vertices.push_back(underSideFthPy);
            attrib.vertices.push_back(underSideFthPz);

            attrib.vertices.push_back(upSideFstPx);
            attrib.vertices.push_back(upSideFstPy);
            attrib.vertices.push_back(upSideFstPz);
            attrib.vertices.push_back(upSideSndPx);
            attrib.vertices.push_back(upSideSndPy);
            attrib.vertices.push_back(upSideSndPz);
            attrib.vertices.push_back(upSideTrdPx);
            attrib.vertices.push_back(upSideTrdPy);
            attrib.vertices.push_back(upSideTrdPz);
            attrib.vertices.push_back(upSideFthPx);
            attrib.vertices.push_back(upSideFthPy);
            attrib.vertices.push_back(upSideFthPz);
        }

        tinyobj::shape_t newShape;
        // Add indeices
        {
            tinyobj::index_t f, s, t;
            f.normal_index = -1;
            f.texcoord_index = -1;
            s.normal_index = -1;
            s.texcoord_index = -1;
            t.normal_index = -1;
            t.texcoord_index = -1;


            f.vertex_index = originalSize;
            s.vertex_index = originalSize + 1;
            t.vertex_index = originalSize + 2;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 1;
            s.vertex_index = originalSize + 2;
            t.vertex_index = originalSize + 3;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize;
            s.vertex_index = originalSize + 1;
            t.vertex_index = originalSize + 4;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 1;
            s.vertex_index = originalSize + 4;
            t.vertex_index = originalSize + 5;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 4;
            s.vertex_index = originalSize + 5;
            t.vertex_index = originalSize + 6;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 4;
            s.vertex_index = originalSize + 6;
            t.vertex_index = originalSize + 7;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 2;
            s.vertex_index = originalSize + 3;
            t.vertex_index = originalSize + 6;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 3;
            s.vertex_index = originalSize + 6;
            t.vertex_index = originalSize + 7;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 1;
            s.vertex_index = originalSize + 2;
            t.vertex_index = originalSize + 5;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize + 2;
            s.vertex_index = originalSize + 5;
            t.vertex_index = originalSize + 6;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize;
            s.vertex_index = originalSize + 3;
            t.vertex_index = originalSize + 7;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);

            f.vertex_index = originalSize;
            s.vertex_index = originalSize + 4;
            t.vertex_index = originalSize + 7;
            newShape.mesh.indices.push_back(f);
            newShape.mesh.indices.push_back(s);
            newShape.mesh.indices.push_back(t);
        };

        for (int i = 0; i < 12; i++) {
            newShape.mesh.num_face_vertices.push_back(3);
            newShape.mesh.material_ids.push_back(-1);
        }
        shapes.push_back(newShape);
    }

    void pointVisualizationPyramid(std::tuple<tinyobj::attrib_t, std::vector<tinyobj::shape_t>, std::vector<tinyobj::material_t>>&, Point3, double)
    {
    }

    std::tuple<std::shared_ptr<SegmentLN>, int> makeProfile(std::vector<SegmentLN> set)
    {
        std::vector<std::shared_ptr<SegmentLN>> pSet;
        for (int i = 0; i < set.size(); i++) {

            auto& t = set.at(i);

            for (int j = 0; j < 4; j++) {

                pSet.push_back(std::make_shared<SegmentLN>(t));

                if (j != 0) {
                    (*(pSet.end() - 2))->setNext(*(pSet.end() - 1));
                    if (j == 3) {
                        (*(pSet.end() - 1))->setNext(*(pSet.end() - 4));
                    }
                }

                t = *t.getNext();
            }
        }

        std::shared_ptr<SegmentLN> ppp = std::make_shared<SegmentLN>();
        std::shared_ptr<SegmentLN> profile = ppp;
        std::shared_ptr<SegmentLN> seg = *pSet.begin();
        size_t totalSeg = 4 * set.size();

        auto lastSeg = seg->getLast();

        while (true) {

            seg->visit();

            Point3 base = seg->getSeg().vertex(0);
            Point3 farp = seg->getSeg().vertex(1);
            int onIntersection = 0, lastIntersection = 0;
            std::vector<std::pair<Point3, std::shared_ptr<SegmentLN>>> intersections;

            // Check if the base is a inner point
            for (int k = 0; k < pSet.size(); k++) {

                auto& tSeg = pSet.at(k);

                if (tSeg->getSeg() != seg->getSeg() && tSeg->getSeg() != lastSeg->getSeg()) {

                    auto result = CGAL::intersection(tSeg->getSeg(), seg->getSeg());
                    auto lastResult = CGAL::intersection(tSeg->getSeg(), lastSeg->getSeg());

                    if (result) {

                        Point3 tp = *boost::get<Point3>(&*result);
                        if (tp != seg->getNext()->getSeg().vertex(0)) {
                            onIntersection += 1;
                            intersections.push_back(std::make_pair(tp, tSeg));
                        }
                    }

                    if (lastResult) {

                        Point3 tp = *boost::get<Point3>(&*lastResult);
                        if (tp != seg->getLast()->getSeg().vertex(0)) {
                            lastIntersection += 1;
                        }
                    }
                }
            }

            // It is not a inner point and no intersection
            if (onIntersection == 0) {

                profile->setNext(seg);
                seg = seg->getNext();
            }
            // Intersection on this side
            else if (onIntersection == 1 && lastIntersection == 0) {

                Point3 tpp = intersections.begin()->first;
                profile->setNext(std::make_shared<SegmentLN>(Segment3(base, tpp)));
                seg = (*intersections.begin()).second;
            }

            else if (onIntersection == 2 && lastIntersection == 0) {

                double fd = CGAL::squared_distance(intersections.begin()->first, base);
                double sd = CGAL::squared_distance((intersections.begin() + 1)->first, base);

                if (seg->ifvisited() == 1) {
                    if (fd > sd) {
                        profile->setNext(std::make_shared<SegmentLN>(
                            Segment3(intersections.begin()->first, farp)));

                    }
                    else {
                        profile->setNext(std::make_shared<SegmentLN>(
                            Segment3((intersections.begin() + 1)->first, farp)));

                    }
                    seg = seg->getNext();
                }
                else {
                    if (fd > sd) {
                        profile->setNext(std::make_shared<SegmentLN>(
                            Segment3(base, (intersections.begin() + 1)->first)));
                        seg = (intersections.begin() + 1)->second;
                    }
                    else {
                        profile->setNext(std::make_shared<SegmentLN>(
                            Segment3(base, intersections.begin()->first)));
                        seg = intersections.begin()->second;
                    }

                }
            }

            else if (onIntersection == 1 && lastIntersection > 0) {

                profile->setNext(std::make_shared<SegmentLN>(
                    Segment3(
                        (*intersections.begin()).first, seg->getSeg().vertex(1))
                    )
                );
                seg = seg->getNext();
            }
            else if (onIntersection == 2 && lastIntersection > 0) {

                double fd = CGAL::squared_distance(intersections.begin()->first, base);
                double sd = CGAL::squared_distance((intersections.begin() + 1)->first, base);

                if (fd < sd) {
                    profile->setNext(std::make_shared<SegmentLN>(
                        Segment3(intersections.begin()->first, (intersections.begin() + 1)->first)));
                    seg = (intersections.begin() + 1)->second;
                }
                else {
                    profile->setNext(std::make_shared<SegmentLN>(
                        Segment3((intersections.begin() + 1)->first, intersections.begin()->first)));
                    seg = intersections.begin()->second;
                }
            }
            else {
                std::cout << "Another?????" << std::endl;
            }
            lastSeg = seg->getLast();
            profile = profile->getNext();

            int curv = 0;
            for (const auto& i : pSet) {
                if (i->ifvisited()) {
                    curv += 1;
                }
            }
            if (curv == pSet.size()) {
                break;
            }
        }
        profile->setNext(ppp->getNext());

        int curv = 0;
        for (const auto& i : pSet) {
            curv += i->ifvisited();
        }
        return std::make_tuple(ppp->getNext(), curv);
    }

    std::tuple<std::shared_ptr<SegmentLN>, int> makeProfileInterval(std::vector<SegmentLN> set)
    {

        auto profile = std::make_shared<SegmentLN>(set.at(0));
        std::shared_ptr<SegmentLN> ppp = profile;
        profile->getLast()->setNext(profile);

        while (set.size() != 1) {

            auto isoShape = std::make_shared<SegmentLN>(*(set.end() - 1));
            isoShape->getLast()->setNext(isoShape);
            set.erase(set.end() - 1);

            auto pro = profile;

            std::shared_ptr<SegmentLN> isop;
            double minDis = 100000000.0;

            int proCount = 0;

            while (true) {

                if (pro->getSeg() == profile->getSeg())
                    proCount += 1;
                if (proCount == 2)
                    break;

                int isoCount = 0;
                auto isoq = isoShape;
                while (isoCount < 2) {

                    if (isoq->getSeg() == isoShape->getSeg()) {
                        isoCount += 1;
                        if (isoCount == 2)
                            break;
                    }

                    Point3
                        proBase = pro->getSeg().vertex(0),
                        proSurvey = pro->getSeg().vertex(1),
                        isoBase = isoq->getSeg().vertex(0),
                        isoSurvey = isoq->getSeg().vertex(1);

                    double avgDis = (CGAL::squared_distance(proBase, isoSurvey) +
                        CGAL::squared_distance(proSurvey, isoBase)) / 2;

                    if (avgDis < minDis) {
                        isop = isoq;
                        ppp = pro;
                        minDis = avgDis;
                    }

                    isoq = isoq->getNext();
                }

                pro = pro->getNext();
            }

            auto zzz = isop->getSeg().vertex(1);
            isop->setSeg(Segment3(isop->getSeg().vertex(0), ppp->getSeg().vertex(1)));
            auto ttt = isop->getNext();
            isop->setNext(ppp->getNext());
            ppp->setSeg(Segment3(ppp->getSeg().vertex(0), zzz));
            ppp->setNext(ttt);

        }

        return std::tuple<std::shared_ptr<SegmentLN>, int>(profile, 1);
    }

    std::vector < std::pair<std::shared_ptr<SegmentLN>, Plane3>>
        getsides(std::vector<Cuboid> cuboids) {
        std::vector<std::pair<std::shared_ptr<SegmentLN>, Plane3>> t;
        for (auto& i : cuboids) {
            auto points = i.getVer();
            auto segments = i.getSeg();
            auto pt = std::make_shared<SegmentLN>(segments.at(0));
            pt->setNext(std::make_shared<SegmentLN>(segments.at(9)));
            pt->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(4)));
            pt->getNext()->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(8)));
            pt->getNext()->getNext()->getNext()->setNext(pt);

            Plane3 pln(points.at(0), points.at(1), points.at(6));

            auto pt2 = std::make_shared<SegmentLN>(segments.at(1));
            pt2->setNext(std::make_shared<SegmentLN>(segments.at(10)));
            pt2->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(5)));
            pt2->getNext()->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(9)));
            pt2->getNext()->getNext()->getNext()->setNext(pt2);

            Plane3 pln2(points.at(1), points.at(2), points.at(7));

            auto pt3 = std::make_shared<SegmentLN>(segments.at(2));
            pt3->setNext(std::make_shared<SegmentLN>(segments.at(11)));
            pt3->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(6)));
            pt3->getNext()->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(10)));
            pt3->getNext()->getNext()->getNext()->setNext(pt3);

            Plane3 pln3(points.at(2), points.at(3), points.at(4));

            auto pt4 = std::make_shared<SegmentLN>(segments.at(3));
            pt4->setNext(std::make_shared<SegmentLN>(segments.at(8)));
            pt4->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(7)));
            pt4->getNext()->getNext()->setNext(std::make_shared<SegmentLN>(segments.at(11)));
            pt4->getNext()->getNext()->getNext()->setNext(pt4);

            Plane3 pln4(points.at(3), points.at(0), points.at(5));

            t.emplace_back(std::make_pair(pt, pln));
            t.emplace_back(std::make_pair(pt2, pln2));
            t.emplace_back(std::make_pair(pt3, pln3));
            t.emplace_back(std::make_pair(pt4, pln4));
        }
        return t;
    }
}