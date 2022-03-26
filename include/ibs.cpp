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

SurfaceMesh
IBSCreating(
	const SurfaceMesh& objl, const SurfaceMesh& objr, const double& bbExpandBias,
	const double& expectedArea
)
{
	SurfaceMesh lmesh = cgaltools::averaged(objl, expectedArea), rmesh = cgaltools::averaged(objr, expectedArea);
	SurfaceMesh ori(objl);
	ori += objr;
	SurfaceMesh obb = cgaltools::obb(ori, bbExpandBias);

	Tree ltree(lmesh.faces().begin(), lmesh.faces().end(), lmesh);
	Tree rtree(rmesh.faces().begin(), rmesh.faces().end(), rmesh);

	PointSet3 lmidpts(true), rmidpts(true);
	lmidpts = generateMidpts(lmesh);
	rmidpts = generateMidpts(rmesh);

	voro::container con(0, 1, 0, 1, 0, 1, 5, 5, 5, false, false, false, 8);

	return obb;
}