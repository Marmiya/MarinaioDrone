#include "calculate_reconstructability.h"
#include <cuda_runtime.h>
#include <cstdlib>
#include <numeric>
#include <thrust/host_vector.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/fill.h>
#include <thrust/scan.h>

__device__ __constant__ double k1 = 32.;
__device__ __constant__ double alpha1 = 3.1415926f / 16;
__device__ __constant__ double k3 = 8.;
__device__ __constant__ double alpha3 = 3.1415926f / 4;
__device__ __constant__ double dmax = 80.;
__device__ __constant__ double v_fov_degree_h = 62.;
__device__ __constant__ double v_fov_degree_v = 42.;

__device__
double
primitiveREC(
	const Eigen::Vector3d lv, const Eigen::Vector3d rv,
	const Eigen::Vector3d v_point, const Eigen::Vector3d v_normal
)
{
	Eigen::Vector3d view_to_point1 = v_point - lv;
	Eigen::Vector3d view_to_point2 = v_point - rv;

	double alpha = acosf(
		view_to_point1.normalized().dot(view_to_point2.normalized())
	);
	double omega1 = 1. / (1 + std::exp(-k1 * (alpha - alpha1)));
	double omega2 = 1 - fminf(
		fmaxf(view_to_point1.norm(), view_to_point2.norm()) / dmax, 1.);
	double omega3 = 1. - 1. / (1 + std::exp(-k3 * (alpha - alpha3)));
	double Theta1 = (-view_to_point1).normalized().dot(v_normal);
	double Theta2 = (-view_to_point2).normalized().dot(v_normal);
	double cosTheta = fminf(Theta1, Theta2);
	double value = omega1 * omega2 * omega3 * cosTheta;
	
	return value;
}


__global__
void
evaluatingREC(
	const Eigen::Vector3d* vs, const Eigen::Vector3d v,
	const Eigen::Vector3d v_point, const Eigen::Vector3d v_normal,
	double* ans, int N)
{
	
	int index = (blockIdx.x * blockDim.x) + threadIdx.x;
	//int index = blockIdx.x + threadIdx.x;
	if (index < N)
	{
		ans[index] = primitiveREC(
			vs[index],v, v_point, v_normal
		);
	}
}


double fundenmentalREC(
	const std::vector<Viewpoint>& vs, const Viewpoint& v,
	const Eigen::Vector3d& v_point, const Eigen::Vector3d& v_normal)
{
	int eleSize = static_cast<int>(vs.size());
	int threadsPerBlock = 256;
	int blockPerGrid = (eleSize + 255) / 256;
	size_t bytes = eleSize * sizeof(double);
	std::vector<double> hAns(eleSize);
	double* ans;
	cudaMalloc(&ans, bytes);

	thrust::device_vector<Eigen::Vector3d> dvss(eleSize);
	for (int i = 0; i < eleSize; i++)
	{
		dvss[i] = vs.at(i).pos_mesh;
	}

	evaluatingREC <<<blockPerGrid, threadsPerBlock >>> (
		thrust::raw_pointer_cast(&dvss[0]), v.pos_mesh, 
		v_point, v_normal, ans, eleSize
		);

	//cudaDeviceSynchronize();
	cudaMemcpy(hAns.data(), ans, bytes, cudaMemcpyDeviceToHost);

	return 1.0;
	//return std::reduce(hAns.begin(), hAns.end());
}

//__device__
__global__
void app(
	Eigen::Vector3d* v, Eigen::Vector3d* pp, Eigen::Vector3d* pn,
	int N, double* anss
)
{
	int indexx = (blockIdx.x * blockDim.x) + threadIdx.x;
	int indexy = (blockIdx.y * blockDim.y) + threadIdx.y;
	if (indexx > indexy && indexx < N && indexy < N)
	{
		anss[indexx + N * indexy] = 
			primitiveREC(v[indexx], v[indexy], *pp, *pn);
	}
}

__global__
void tREC(
	Eigen::Vector3d* v, Eigen::Vector3d* pp, Eigen::Vector3d* pn, 
	int* vis, int* presum, double* ans, int pN
)
{
	int index = (blockIdx.x * blockDim.x) + threadIdx.x;
	
	if (index < pN)
	{
		double z = 0.;
		int tsize = presum[index + 1] - presum[index];
		for (int i = 0; i < tsize; i++)
		{
			Eigen::Vector3d l = v[vis[presum[index] + i]];
			for (int j = i + 1; j < tsize; j++)
			{
				Eigen::Vector3d r = v[vis[presum[index] + j]];
				double cur = primitiveREC(l, r, pp[index], pn[index]);
				if (cur > 0.)
				{
					z += cur;
				}
			}
		}
		
		ans[index] = z;
		
		/*Eigen::Vector3d* t;

		cudaMalloc(&t, tsize * sizeof(Eigen::Vector3d));*/

		/*double* anss;
		cudaMalloc(&anss, tsize * tsize);
		int unit = 16;
		dim3 tpb(unit, unit);
		dim3 bpg((tsize + unit - 1) / unit, (tsize + unit - 1) / unit);
		app <<<bpg, tpb >>> (thrust::raw_pointer_cast(&t[0]), &pp[index], &pn[index], tsize, anss);*/
		//ans[index] = thrust::reduce(anss, anss + tsize * tsize);
		
	}
}

std::vector<double>
totalRECv(
	const std::vector<Viewpoint>& trajectory,
	const std::vector<Eigen::Vector3d>& ptsp, const std::vector<Eigen::Vector3d>& ptsn,
	std::vector<std::vector<int>>& visibility
	)
{
	int viewsSize = static_cast<int>(trajectory.size());
	int ptsSize = static_cast<int>(ptsp.size());
	int tpb = 512;
	int bpg = (ptsSize + tpb - 1) / tpb;

	thrust::device_vector<Eigen::Vector3d> vwsPos(viewsSize);
	thrust::device_vector<Eigen::Vector3d> ptsP(ptsSize);
	thrust::device_vector<Eigen::Vector3d> ptsN(ptsSize);
	int* vnum = (int*)malloc((ptsSize + 1) * sizeof(int));
	thrust::device_vector<int> gvnum(ptsSize + 1);
	thrust::host_vector<int> indices;
	double* ans;
	std::vector<double> hAns(ptsSize);

#pragma omp parallel for
	for (int i = 0; i < viewsSize; i++)
	{
		vwsPos[i] = trajectory.at(i).pos_mesh;
	}

#pragma omp parallel for
	for (int i = 0; i < ptsSize; i++)
	{
		ptsP[i] = ptsp.at(i);
		ptsN[i] = ptsn.at(i);
	}

	vnum[0] = 0;
	for (int i = 1; i < ptsSize + 1; i++)
	{
		vnum[i] = visibility.at(i - 1).size();
	}
	thrust::inclusive_scan(vnum, vnum + ptsSize + 1, vnum);
	
	thrust::copy(vnum, vnum + ptsSize + 1, gvnum.begin());
	free(vnum);

	for (int i = 0; i < ptsSize; i++)
	{
		for (const auto& j : visibility.at(i))
		{
			indices.push_back(j);
		}
	}
	thrust::device_vector<int> vis(indices.begin(), indices.end());

	cudaMalloc(&ans, ptsSize * sizeof(double));

	tREC <<<bpg, tpb >>> (
		thrust::raw_pointer_cast(&vwsPos[0]),
		thrust::raw_pointer_cast(&ptsP[0]), thrust::raw_pointer_cast(&ptsN[0]),
		thrust::raw_pointer_cast(&vis[0]), thrust::raw_pointer_cast(&gvnum[0]),
		thrust::raw_pointer_cast(&ans[0]),
		ptsSize
		);

	cudaDeviceSynchronize();

	cudaMemcpy(hAns.data(), ans, ptsSize * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(ans);
	return hAns;
}

double
totalREC(
	const std::vector<Viewpoint>& trajectory,
	const std::vector<Eigen::Vector3d>& ptsp, const std::vector<Eigen::Vector3d>& ptsn,
	std::vector<std::vector<int>>& visibility
)
{
	std::vector<double> t = totalRECv(
		trajectory,
		ptsp, ptsn,
		visibility
	);
	return 1.;
	//return std::reduce(t.begin(), t.end());
}