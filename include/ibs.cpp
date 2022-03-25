#include "ibs.h"

SurfaceMesh
IBSCreating(const SurfaceMesh& objl, const SurfaceMesh& objr)
{

	SurfaceMesh lmesh = cgaltools::averaged(objl, 1.), rmesh = cgaltools::averaged(objr, 1.);
	
	return lmesh;
}