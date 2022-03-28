#include "ibs.h"

PointSet3
generateMidpts(const SurfaceMesh& mesh)
{
	PointSet3 midpts(true);
	int FaceSize = static_cast<int>(mesh.number_of_faces());

#pragma omp parallel for
	for (int faceID = 0; faceID < FaceSize; faceID++)
	{
		SMFI curFI(faceID);
		CGAL::Vertex_around_face_iterator<SurfaceMesh> vbegin, vend;
		boost::tie(vbegin, vend) = vertices_around_face(mesh.halfedge(curFI), mesh);
		SurfaceMesh::vertex_index ep1VI = *vbegin, ep2VI = *(++vbegin), ep3VI = *(++vbegin);
		Point3 p1 = mesh.point(ep1VI), p2 = mesh.point(ep2VI), p3 = mesh.point(ep3VI);
		Point3 midpoint = CGAL::midpoint(p1, CGAL::midpoint(p2, p3));
		Vector3 direction = CGAL::cross_product((p2 - p1), (p3 - p2));
		direction /= CGAL::sqrt(direction.squared_length());
#pragma omp critical
		{
			midpts.insert(midpoint, direction);
		}
	}

	return midpts;
}

void
draw_polygon(
	FILE* fp, std::vector<int>& f_vert, std::vector<double>& v, int j)
{
	static char s[6][128];
	int k, l, n = f_vert[j];

	// Create POV-Ray vector strings for each of the vertices
	for (k = 0; k < n; k++) {
		l = 3 * f_vert[j + k + 1];
		sprintf(s[k], "<%g,%g,%g>", v[l], v[l + 1], v[l + 2]);

	}

	// Draw the interior of the polygon
	fputs("union{\n", fp);
	for (k = 2; k < n; k++) fprintf(fp, "\ttriangle{%s,%s,%s}\n", s[0], s[k - 1], s[k]);
	fputs("\ttexture{t1}\n}\n", fp);

	// Draw the outline of the polygon
	fputs("union{\n", fp);
	for (k = 0; k < n; k++) {
		l = (k + 1) % n;
		fprintf(fp, "\tcylinder{%s,%s,r}\n\tsphere{%s,r}\n",
			s[k], s[l], s[l]);

	}
	fputs("\ttexture{t2}\n}\n", fp);

}

inline bool
ifneighbor(
	std::vector<size_t> ptsSize,
	const size_t& a, const size_t& b)
{
	size_t aclass = ptsSize.size();
	size_t total = 0;
	for (size_t i = 0; i < ptsSize.size(); i++)
	{
		total += ptsSize.at(i);
		if (total > a)
		{
			aclass = i;
			break;
		}
	}
	if (aclass == ptsSize.size())
	{
		throw;
	}
	size_t totalbb = std::reduce(ptsSize.begin(), ptsSize.begin() + aclass);
	size_t totalba = totalbb + ptsSize.at(aclass);
	if (b >= totalbb && b < totalba)
	{
		return true;
	}
	else
	{
		return false;
	}
}

SurfaceMesh
IBSCreating(
	const std::vector<SurfaceMesh>& objs,
	const double& bbExpandBias, const double& expectedArea
)
{
	SurfaceMesh ori;
	SurfaceMesh ans;

	size_t objsSize = objs.size();
	
	std::vector<SurfaceMesh> meshs(objsSize);

	for (size_t i = 0; i < objsSize; i++)
	{
		meshs.at(i) = cgaltools::averaged(objs.at(i), expectedArea);
		ori += objs.at(i);
	}

	SurfaceMesh obb = cgaltools::obb(ori, bbExpandBias);

	std::vector<PointSet3> ptss(objsSize, true);

	for (size_t i = 0; i < objsSize; i++)
	{
		ptss.at(i) = generateMidpts(meshs.at(i));
	}

	std::vector<size_t> midptsSize(objsSize);
	for (size_t i = 0; i < objsSize; i++)
	{
		midptsSize.at(i) = ptss.at(i).size();
	}

	double ax = 363363., ay = ax, az = ay;
	double bx = -363363., by = bx, bz = by;
	for (const auto& i : obb.points())
	{
		if (i.x() < ax)
		{
			ax = i.x();
		}
		if (i.x() > bx)
		{
			bx = i.x();
		}
		if (i.y() < ay)
		{
			ay = i.y();
		}
		if (i.y() > by)
		{
			by = i.y();
		}
		if (i.z() < az)
		{
			az = i.z();
		}
		if (i.z() > bz)
		{
			bz = i.z();
		}
	}

	voro::container con(ax, bx, ay, by, az, bz, 50, 50, 50, false, false, false, 8);

	size_t startIndex = 0;
	for (size_t i = 0; i < objsSize; i++)
	{
		for (size_t j = 0; j < midptsSize.at(i); j++)
		{
			con.put(j + startIndex, ptss.at(i).point(j).x(), ptss.at(i).point(j).y(), ptss.at(i).point(j).z());
		}
		startIndex += midptsSize.at(i);
	}

	double x, y, z;
	int id;
	voro::voronoicell_neighbor c;
	std::vector<int> neigh, f_vert;
	std::vector<double> v;
	
	voro::c_loop_all cl(con);

	size_t i, j;
	if (cl.start()) 
		do if (con.compute_cell(c, cl)) {
			cl.pos(x, y, z); id = cl.pid();

			// Gather information about the computed Voronoi cell
			c.neighbors(neigh);
			c.face_vertices(f_vert);
			c.vertices(x, y, z, v);
			
			// Loop over all faces of the Voronoi cell
			for (i = 0, j = 0; i < neigh.size(); i++) 
			{
				if (neigh[i] > id && !ifneighbor(midptsSize, id, neigh[i]))
					
				{

					int k, l, n = f_vert[j];
					std::vector<SMVI> ptsVI;

					for (k = 0; k < n; k++) {
						l = 3 * f_vert[j + k + 1];
						ptsVI.push_back(ans.add_vertex(Point3(v[l], v[l + 1], v[l + 2])));
					}

					ans.add_face(ptsVI);

				}
				
				// Skip to the next entry in the face vertex list
				j += f_vert[j] + 1;

			}

		} while (cl.inc());

	//con.draw_particles("C:\\Users\\mic\\Documents\\gnuplot\\random_points_p.gnu");
	//con.draw_cells_gnuplot("C:\\Users\\mic\\Documents\\gnuplot\\random_points_v.gnu");
	return ans;
}

SurfaceMesh
IBSCreatingWithSenceBB(
	const std::vector<SurfaceMesh>& objs,
	const double& bbExpandBias, const double& expectedArea
)
{
	SurfaceMesh ori;
	SurfaceMesh ans;

	size_t objsSize = objs.size();

	for (size_t i = 0; i < objsSize; i++)
	{
		ori += objs.at(i);
	}
	SurfaceMesh obb = cgaltools::obb(ori, bbExpandBias);

	double ax = 363363., ay = ax, az = ay;
	double bx = -363363., by = bx, bz = by;
	for (const auto& i : obb.points())
	{
		if (i.x() < ax)
		{
			ax = i.x();
		}
		if (i.x() > bx)
		{
			bx = i.x();
		}
		if (i.y() < ay)
		{
			ay = i.y();
		}
		if (i.y() > by)
		{
			by = i.y();
		}
		if (i.z() < az)
		{
			az = i.z();
		}
		if (i.z() > bz)
		{
			bz = i.z();
		}
	}
	bz += 5.;
	SurfaceMesh sencebb;
	std::vector<SMVI> ptsVI;
	ptsVI.push_back(sencebb.add_vertex(Point3(ax, ay, az)));
	ptsVI.push_back(sencebb.add_vertex(Point3(bx, ay, az)));
	ptsVI.push_back(sencebb.add_vertex(Point3(bx, by, az)));
	ptsVI.push_back(sencebb.add_vertex(Point3(ax, by, az)));
	ptsVI.push_back(sencebb.add_vertex(Point3(ax, by, bz)));
	ptsVI.push_back(sencebb.add_vertex(Point3(ax, ay, bz)));
	ptsVI.push_back(sencebb.add_vertex(Point3(bx, ay, bz)));
	ptsVI.push_back(sencebb.add_vertex(Point3(bx, by, bz)));
	std::vector<SMVI> f;
	f.push_back(ptsVI.at(0));
	f.push_back(ptsVI.at(1));
	f.push_back(ptsVI.at(6));
	f.push_back(ptsVI.at(5));
	sencebb.add_face(f);
	f.clear();
	f.push_back(ptsVI.at(1));
	f.push_back(ptsVI.at(2));
	f.push_back(ptsVI.at(7));
	f.push_back(ptsVI.at(6));
	sencebb.add_face(f);
	f.clear();
	f.push_back(ptsVI.at(2));
	f.push_back(ptsVI.at(3));
	f.push_back(ptsVI.at(4));
	f.push_back(ptsVI.at(7));
	sencebb.add_face(f);
	f.clear();
	f.push_back(ptsVI.at(3));
	f.push_back(ptsVI.at(0));
	f.push_back(ptsVI.at(5));
	f.push_back(ptsVI.at(4));
	sencebb.add_face(f);
	f.clear();
	f.push_back(ptsVI.at(4));
	f.push_back(ptsVI.at(5));
	f.push_back(ptsVI.at(6));
	f.push_back(ptsVI.at(7));
	sencebb.add_face(f);
	sencebb = cgaltools::averaged(sencebb, 2.);
	std::vector<SurfaceMesh> tobjs = objs;
	tobjs.push_back(sencebb);

	return IBSCreating(tobjs, bbExpandBias, expectedArea);
}