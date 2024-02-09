#include "types_gpu.hpp"


PointCloudGPU::PointCloudGPU(velodyne_decoder::PointCloud& point_cloud) {

    size_t bytes = MAX_NUM_POINTS * sizeof(float);

    cudaMalloc(&d_point_cloud_ptr, bytes);

    int idx_field = 0;
    for(const auto &point : point_cloud) {
        point_cloud_array[idx_field++] = point.x;
        point_cloud_array[idx_field++] = point.y;
        point_cloud_array[idx_field++] = point.z;
        point_cloud_array[idx_field++] = point.intensity; 
    }

    cudaMemcpy(d_point_cloud_ptr, point_cloud_array.data(), bytes, cudaMemcpyHostToDevice);
}

PointCloudGPU::~PointCloudGPU() {
    cudaFree(d_point_cloud_ptr);
}

void PointCloudGPU::computeRangeArray(std::array<float, MAX_NUM_POINTS>& ranges) {
    computeRangeKernel(d_point_cloud_ptr, ranges.data());
}