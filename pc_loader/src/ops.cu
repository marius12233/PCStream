#ifndef _OPS_
#define _OPS_

#include "types_gpu.hpp"

// Kernel definition
__global__ void computeRange(const float* point_cloud, float* ranges)
{
    int idx = threadIdx.x * NUM_FIELDS ;

    float range = 0;
    for(int i=0; i < NUM_FIELDS - 1; i++) {
        auto coord = point_cloud[idx * NUM_FIELDS + i];
        range += coord * coord;
    }
    ranges[idx] = std::sqrt(range);
}

void computeRangeKernel(const float* point_cloud_gpu_raw_data, float* ranges) {
    int numBlocks = 1;
    dim3 threadsPerBlock(1, MAX_NUM_POINTS);
    computeRange<<<numBlocks, threadsPerBlock>>>(point_cloud_gpu_raw_data, ranges);
}

#endif