#include <cuda_runtime.h>
#include "velodyne_decoder/types.h"

static constexpr std::size_t MAX_NUM_POINTS = 300000;
static constexpr std::size_t NUM_FIELDS = 4;

#define gpuErrchk(ans) { gpuAssert((ans), __FILE__, __LINE__); }
inline void gpuAssert(cudaError_t code, const char *file, int line, bool abort=true)
{
   if (code != cudaSuccess) 
   {
      fprintf(stderr,"GPUassert: %s %s %d\n", cudaGetErrorString(code), file, line);
      if (abort) exit(code);
   }
}

class PointCloudGPU {
    public:
        PointCloudGPU(velodyne_decoder::PointCloud&);
        ~PointCloudGPU();
        void computeRangeArray(std::array<float, MAX_NUM_POINTS>& ranges);
        std::array<float, MAX_NUM_POINTS * NUM_FIELDS>& toHost();
    
    private:
        std::array<float, MAX_NUM_POINTS * NUM_FIELDS> point_cloud_array;
        float *d_point_cloud_ptr;
        size_t num_points{0};

};

void computeRangeKernel(const float* point_cloud_gpu_raw_data, float* ranges, size_t num_current_points);


