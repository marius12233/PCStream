// Copyright 2013 Velodyne Acoustics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    PacketFileSender.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME PacketFileSender -
// .SECTION Description
// This program reads a pcap file and sends the packets using UDP.


#include <string>
#include <cstdlib>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include "velodyne_reader.hpp"
#include "types_gpu.hpp"
#include <math.h>


int main(int argc, char* argv[])
{
    

    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <packet file>" << std::endl;
        return 1;
    }

    std::string filename = argv[1];
    std::string calibration_file;
    if(argc < 3) {
        calibration_file = "calibrations/VLP-16.yml";
    } else {
        calibration_file = argv[2];
    }
    VelodyneReaderAdapter velodyne_reader{filename, calibration_file};
    velodyne_decoder::PointCloud pcl;
    velodyne_reader.read_scan(pcl);
    std::cout << pcl[0].intensity << std::endl;
    std::cout << "PCL ori point 0: " << pcl[0].x << " " << 
                                        pcl[0].y << " " << 
                                        pcl[0].z << std::endl;

    PointCloudGPU point_cloud_gpu{pcl};

    auto& point_cloud_arr = point_cloud_gpu.toHost();

    std::cout << "PCL arr point 0: " << point_cloud_arr[0] << " " << 
                                        point_cloud_arr[1] << " " << 
                                        point_cloud_arr[2] << std::endl;

    std::array<float, MAX_NUM_POINTS> ranges;
    point_cloud_gpu.computeRangeArray(ranges);

    std::cout << "Range size: " << pcl.size() << std::endl;

    for(int i=0; i < pcl.size(); i++) {
        float cpu_range = std::sqrt(pcl[i].x * pcl[i].x + 
        pcl[i].y * pcl[i].y +
        pcl[i].z * pcl[i].z);
        assert(cpu_range == ranges[i]);

    }


return 0;
}
