#include "CalRec.cuh"

__device__ double k1 = 32.;
__device__ double alpha1 = 0.19634954;
__device__ double k3 = 8.;
__device__ double alpha3 = 0.78539815;

__device__ double primitiveRec(
	double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis
)
{
	double3 view_to_point1 = v_point - lv;
	double3 view_to_point2 = v_point - rv;

	const double alpha = acosf(
		dot(normalizeD3(view_to_point1), normalizeD3(view_to_point2)));

	const double omega1 = 1. / (1 + std::exp(-k1 * (alpha - alpha1)));
	const double omega2 = 1. - fminf(
		fmaxf(norm(view_to_point1), norm(view_to_point2)) / maxdis, 1.);
	const double omega3 = 1. - 1. / (1 + std::exp(-k3 * (alpha - alpha3)));

	const double Theta1 = -dot(normalizeD3(v_normal), normalizeD3(view_to_point1));
	const double Theta2 = -dot(normalizeD3(v_normal), normalizeD3(view_to_point2));
	const double cosTheta = fminf(Theta1, Theta2);
	
	return omega1 * omega2 * omega3 * cosTheta;
}

__global__ void computeSingleRec(
	double3* vs, double3 v, double3 ptspos, double3 ptsnormal,
	double maxdis, double* ans, int N)
{

	int threadID = (blockIdx.x * blockDim.x) + threadIdx.x;
	if (threadID < N)
	{
		ans[threadID] = primitiveRec(vs[threadID], v, ptspos, ptsnormal, maxdis);
	}
}

double singleRec(
	const std::vector<double3>& viewsPos, const double3& v,
	double3 ptspos, double3 ptsnormal,
	const int& tpb, double maxdis
)
{
	int eleSize = static_cast<int>(viewsPos.size());
	int bpg = (eleSize + tpb - 1) / tpb;

	size_t bytes = eleSize * sizeof(double);
	double* ans;
	cudaMalloc(&ans, bytes);
	std::vector<double> hAns(eleSize);

	thrust::device_vector<double3> viewsPosCu(viewsPos.begin(), viewsPos.end());

	computeSingleRec <<<bpg, tpb>>>(
		thrust::raw_pointer_cast(&viewsPosCu[0]), v,
		ptspos, ptsnormal, maxdis, ans, eleSize
		);

	cudaDeviceSynchronize();
	cudaMemcpy(hAns.data(), ans, bytes, cudaMemcpyDeviceToHost);

	return std::reduce(hAns.begin(), hAns.end());
}

__global__ void tRec(
	double3* v, double3* pp, double3* pn, double maxdis,
	int* vis, int* presum, double* ans, int pN
)
{
	int threadID = (blockIdx.x * blockDim.x) + threadIdx.x;

	if (threadID < pN)
	{
		double z = 0.;
		int tsize = presum[threadID + 1] - presum[threadID];
		for (int i = 0; i < tsize; i++)
		{
			double3 l = v[vis[presum[threadID] + i]];
			for (int j = i + 1; j < tsize; j++)
			{
				double3 r = v[vis[presum[threadID] + j]];
				double cur = primitiveRec(l, r, pp[threadID], pn[threadID], maxdis);
				if (cur > 0.)
				{
					z += cur;
				}
			}
		}
		ans[threadID] = z;
	}
}


std::vector<double> totalRecv(
	const std::vector<double3>& viewsPos,
	const std::vector<double3>& ptsPos, const std::vector<double3>& ptsNormal,
	std::vector<std::vector<int>>& visibility, const int& tpb,
	const double& maxdis, const double& hfov, const double& vfov
)
{
	int viewsSize = static_cast<int>(viewsPos.size());
	int ptsSize = static_cast<int>(ptsPos.size());
	const int bpg = (ptsSize + tpb - 1) / tpb;

	double* ans;
	std::vector<double> hAns(ptsSize);

	thrust::device_vector<double3> vwsPos(viewsPos.begin(), viewsPos.end());
	thrust::device_vector<double3> ptsP(ptsPos.begin(), ptsPos.end());
	thrust::device_vector<double3> ptsN(ptsNormal.begin(), ptsNormal.end());

	int* vNumH = (int*)malloc((ptsSize + 1) * sizeof(int));
	vNumH[0] = 0;
	for (int i = 1; i < ptsSize + 1; i++)
	{
		vNumH[i] = visibility.at(i - 1).size();
	}
	thrust::inclusive_scan(vNumH, vNumH + ptsSize + 1, vNumH);
	thrust::device_vector<int> vNumD(vNumH, vNumH + ptsSize + 1);

	thrust::host_vector<int> indices;
	for (int i = 0; i < ptsSize; i++)
	{
		for (const auto& j : visibility.at(i))
		{
			indices.push_back(j);
		}
	}
	thrust::device_vector<int> indicesCu(indices.begin(), indices.end());

	cudaMalloc(&ans, ptsSize * sizeof(double));

	tRec<<<bpg,tpb>>>(
		thrust::raw_pointer_cast(&vwsPos[0]), thrust::raw_pointer_cast(&ptsP[0]), thrust::raw_pointer_cast(&ptsN[0]),
		maxdis,
		thrust::raw_pointer_cast(&indicesCu[0]), thrust::raw_pointer_cast(&vNumD[0]),
		thrust::raw_pointer_cast(&ans[0]), ptsSize
		);

	cudaDeviceSynchronize();

	cudaMemcpy(hAns.data(), ans, ptsSize * sizeof(double), cudaMemcpyDeviceToHost);
	cudaFree(ans);
	return hAns;
}

__device__ double3 operator-(const double3& l, const double3& r)
{
	return make_double3(l.x - r.x, l.y - r.y, l.z - r.z);
}

__device__ double3 normalizeD3(const double3& a)
{
	const float length = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	return make_double3(a.x / length, a.y / length, a.z / length);
}

__device__ float dot(const double3& l, const double3& r)
{
	return l.x * r.x + l.y * r.y + l.z * r.z;
}

__device__ float norm(const double3& a)
{
	return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

__global__ void primitiveTest(
	double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis, double* ans
)
{
	*ans = primitiveRec(lv, rv, v_point, v_normal, maxdis);
}

double pt(double3 lv, double3 rv, double3 v_point, double3 v_normal, double maxdis)
{
	double* ans;
	double* ansCu;
	ans = (double*)malloc(sizeof(double));
	cudaMalloc(&ansCu, sizeof(double));
	primitiveTest <<<1, 1 >>> (lv, rv, v_point, v_normal, maxdis, ansCu);
	cudaMemcpy(ans, ansCu, sizeof(double), cudaMemcpyDeviceToHost);
	return ans[0];
}