#include "intersection_tools.h"
#include <voro++.hh>


extern std::array<double, 36> roundedDirection;

PointSet3
generateMidpts(const SurfaceMesh& mesh);

void
draw_polygon(
	FILE* fp, std::vector<int>& f_vert, std::vector<double>& v, int j);

inline bool
ifneighbor(
	std::vector<size_t> ptsSize,
	const size_t& a, const size_t& b);

SurfaceMesh
IBSCreating(
	const std::vector<SurfaceMesh>& objs, 
	const double& bbExpandBias,	const double& expectedArea
);

SurfaceMesh
IBSCreatingWithSenceBB(
	const std::vector<SurfaceMesh>& objs,
	const double& bbExpandBias, const double& expectedArea,
	const double& safeDis
);

bool
IBSviewsGeneration(
	const Tree& tree, const Point3& midp, Point3& ansp, Vector3& ansv,
	const std::pair<Point3, double>& monitoringp, const double& safeDis
);

SurfaceMesh
IBSTriangulation(
	const SurfaceMesh& IBS
);

/*
 * The returned point set's position of points are views' position,
 * and their normals are views' direction.
 */
PointSet3 IBSviewNet(
	const SurfaceMesh& IBS, const std::vector<SurfaceMesh>& meshs,
	const double& safeDis, const double& safeHeight, 
	const double& verticalStep, const double& horizontalStep
);