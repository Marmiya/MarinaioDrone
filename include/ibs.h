#include "cgal_tools.h"
#include <voro++.hh>

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