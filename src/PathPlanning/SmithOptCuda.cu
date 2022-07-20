#include "SmithOptCuda.cuh"

std::vector<std::pair<double, double>>viewsOffsetC;
double verStripAngle;
double horiStripAngle;
int tpb = 256;

__global__ void initOffset(
	double2* offset, int cvn, 
	int hsn, double vsa, double hsa)
{
	int threadID = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadID < cvn)
	{
		int vid = threadID / hsn;
		int hid = threadID % hsn;
		offset[threadID] = make_double2(vid * vsa, hid * hsa);
	}

}

void initOffsetInf(
	int bn, int tpb, double2* offset, 
	int cvn, int hsn, double vsa, double hsa)
{
	initOffset<<<bn,tpb>>>(offset, cvn, hsn, vsa, hsa);
}

__global__ void conveyDataDouble2(double2* data, int length, double* cpuDataFst, double* cpuDataSnd)
{
	int threadID = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadID < length)
	{
		cpuDataFst[threadID] = data[threadID].x;
		cpuDataSnd[threadID] = data[threadID].y;
	}
}

void conveyDataDouble2Inf(double2* data, int length, double* cpuDataFst, double* cpuDataSnd)
{
	int blocksNum = length / tpb + 1;
	conveyDataDouble2<<<blocksNum, tpb>>>(data, length, cpuDataFst, cpuDataSnd);
}
