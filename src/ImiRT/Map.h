#pragma once

#include "model_tools.h"

using string = std::string;
class GTMap
{
public:
	SurfaceMesh surfaceMesh;
	PointSet3 samplePts;

	GTMap() = delete;
	GTMap(const string& modelPath, const double& ID, const double& UID, const double& filterHeight);
	GTMap(const string& modelPath, const string& ptsPath);
};