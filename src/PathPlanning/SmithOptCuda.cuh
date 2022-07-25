#pragma once
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <type_traits>
#include <vector>
#include <utility>
#include <optix.h>
#include <optix_stubs.h>
#include <optix_stack_size.h>
#include <optix_function_table_definition.h>

extern std::vector<std::pair<double, double>> viewsOffsetC;
extern double verStripAngle;
extern double horiStripAngle;

extern int tpb;


__device__ bool vis();

__device__ double4 maximizeScore();

__device__ void NelderMeadMethod();

__global__ void SmithAdj();

__global__ void initOffset(double2* offset, int cvn, int hsn, double vsa, double hsa);

void initOffsetInf(int bn, int tpb, double2* offset, int cvn, int hsn, double vsa, double hsa);

__global__ void conveyDataDouble2(double2* data, int length, double* cpuDataFst, double* cpuDataSnd);

void conveyDataDouble2Inf(double2* data, int length, double* cpuDataFst, double* cpuDataSnd);



template <typename T, typename U>
bool conveyFromCuda(T* cudaData, U& cpuData, int length)
{
	int amplifierCoefficient;
	if (std::is_same_v<double2, typename std::decay_t<T>> &&
		std::is_same_v<std::vector<std::pair<double, double>>, typename std::decay_t<U> >)
	{
		amplifierCoefficient = 2;
		const int cpuLength = amplifierCoefficient * length;
		double* cpuDataFstH, * cpuDataSndH;
		double* cpuDataFstD, * cpuDataSndD;
		size_t bitSize = length * sizeof(double);
		cpuDataFstH = (double*)malloc(bitSize);
		cpuDataSndH = (double*)malloc(bitSize);
		cudaMalloc((void**)&cpuDataFstD, bitSize);
		cudaMalloc((void**)&cpuDataSndD, bitSize);

		conveyDataDouble2Inf(cudaData, length, cpuDataFstD, cpuDataSndD);

		cudaMemcpy(cpuDataFstH, cpuDataFstD, bitSize, cudaMemcpyDeviceToHost);
		cudaMemcpy(cpuDataSndH, cpuDataSndD, bitSize, cudaMemcpyDeviceToHost);

		if (cpuData.capacity() < length)
		{
			cpuData = std::move(std::vector<std::pair<double,double>>(length));
		}
		#pragma omp parallel for
		for (int i = 0; i < length; i++)
		{
			cpuData[i] = std::make_pair(cpuDataFstH[i], cpuDataSndH[i]);
		}
		free(cpuDataFstH);
		free(cpuDataSndH);
		cudaFree(cpuDataFstD);
		cudaFree(cpuDataSndD);
		return true;
	}
	else
	{
		return false;
	}

}