#include "cgal_tools.h"
#include <voro++.hh>

PointSet3
generateMidpts(const SurfaceMesh& mesh);

SurfaceMesh
IBSCreating(
	const SurfaceMesh& objl, const SurfaceMesh& objr, const double& bbExpandBias,
	const double& expectedArea
);