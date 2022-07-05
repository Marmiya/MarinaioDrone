#include "Drone.h"

Drone::Drone(
	const double& viewDis, const double& safeDis,
	const double& vFov, const double& hFov,
	const double& vOverlap, const double& hOverlap) :
	viewDis(viewDis), safeDis(safeDis), vFov(vFov), hFov(hFov), vOverlap(vOverlap), hOverlap(hOverlap)
{

}
