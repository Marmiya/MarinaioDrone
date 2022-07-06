#pragma once

#include "intersection_tools.h"
#include "model_tools.h"

using string = std::string;
class GTMap
{
public:
	SurfaceMesh surfaceMesh;
	RTCScene rtcScene;
	PointSet3 samplePts;
	std::tuple<double, double, double, double> range;
	std::vector<double> xGridPoint;
	std::vector<double> yGridPoint;

	GTMap() = delete;
	/*
	 * Arguments:
	 * Important Density, unimportant Density
	 */
	GTMap(const string& modelPath, const double& ID, const double& UID, const double& filterHeight);
	GTMap(const string& modelPath, const string& ptsPath);

	/*
	 * Arguments:
	 * CCPPCellLength
	 */
	void initialization(const double& CCL);

};