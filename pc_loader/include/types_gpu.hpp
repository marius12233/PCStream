#include <cuda_runtime.h>
#include "velodyne_decoder/types.h"

static constexpr std::size_t MAX_NUM_POINTS = 300000;
static constexpr std::size_t NUM_FIELDS = 4;


class PointCloudGPU {
    public:
        PointCloudGPU(velodyne_decoder::PointCloud&);
        ~PointCloudGPU();
        void computeRangeArray(std::array<float, MAX_NUM_POINTS>& ranges);
    
    private:
        std::array<float, MAX_NUM_POINTS * NUM_FIELDS> point_cloud_array;
        float *d_point_cloud_ptr;

};

void computeRangeKernel(const float* point_cloud_gpu_raw_data, float* ranges);


