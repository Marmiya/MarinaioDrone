#pragma once
#include <numeric>
#include <vector>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <cstdlib>

#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/scan.h>

extern __device__ double k1;
extern __device__ double alpha1;
extern __device__ double k3;
extern __device__ double alpha3;

__device__ double primitiveRec(
	double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis
);

__global__ void computeSingleRec(
	double3* vs, double3 v, double3 ptspos, double3 ptsnormal,
	double maxdis, double* ans, int N);

double singleRec(
	const std::vector<double3>& viewsPos, const double3& v,
	double3 ptspos, double3 ptsnormal,
	const int& tpb, double maxdis
);

__global__ void tRec(
	double3* v, double3* pp, double3* pn, double maxdis,
	int* vis, int* presum, double* ans, int pN
);

std::vector<double> totalRecv(
	const std::vector<double3>& viewsPos,
	const std::vector<double3>& ptsPos, const std::vector<double3>& ptsNormal,
	std::vector<std::vector<int>>& visibility, const int& tpb,
	const double& maxdis, const double& hfov, const double& vfov
);

__device__ double3 operator-(const double3& l, const double3& r);
__device__ double3 normalizeD3(const double3& a);
__device__ float norm(const double3& a);
__device__ float dot(const double3& l, const double3& r);

__global__ void primitiveTest(
	double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis
);

double pt(double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis);

template<typename T,typename Fuc,typename... Args>
auto resultFromCu(const T& blockNum, const T& tpb, Fuc f, Args... args)
{

};

