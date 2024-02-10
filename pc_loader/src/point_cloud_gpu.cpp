#include "types_gpu.hpp"
#include <iostream>

PointCloudGPU::PointCloudGPU(velodyne_decoder::PointCloud& point_cloud) {
    num_points = point_cloud.size();
    const size_t bytes = num_points * NUM_FIELDS * sizeof(float);

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

std::array<float, MAX_NUM_POINTS * NUM_FIELDS>& PointCloudGPU::toHost() {
    cudaMemcpy(point_cloud_array.data(), d_point_cloud_ptr, num_points * NUM_FIELDS * sizeof(float), cudaMemcpyDeviceToHost);
    return point_cloud_array;
}

void PointCloudGPU::computeRangeArray(std::array<float, MAX_NUM_POINTS>& ranges) {
    float* d_ranges_ptr;
    const size_t bytes = num_points * sizeof(float);
    gpuErrchk(cudaMalloc(&d_ranges_ptr, bytes));

    computeRangeKernel(d_point_cloud_ptr, d_ranges_ptr, bytes); //point_cloud_array.size());

    gpuErrchk(cudaMemcpy(ranges.data(), d_ranges_ptr, bytes, cudaMemcpyDeviceToHost));
    gpuErrchk(cudaFree(d_ranges_ptr));

}