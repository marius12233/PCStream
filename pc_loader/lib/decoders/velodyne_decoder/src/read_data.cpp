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
#include "velodyne_decoder/input.h"
#include "velodyne_decoder/packet_decoder.h"
#include "velodyne_decoder/config.h"
#include "velodyne_decoder/stream_decoder.h"
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>


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

    velodyne_decoder::InputPCAP packetReader(velodyne_decoder::DATA_PORT_NUMBER, 10.0, filename, true);
    velodyne_decoder::Config config{"VLP-16", calibration_file};
    config.rpm = 600; // 600 / 60 = 10 frames per second (Hz) at which the packets are collected
    velodyne_decoder::StreamDecoder decoder{config};
    double timeSinceStart = 0;
    int nscans = 100;


    for (int i = 0; i < nscans; ++i) {
        bool is_scan_full = false;
        while (!is_scan_full) {
            velodyne_decoder::VelodynePacket pkt;
            // keep reading until full packet received
            int rc = packetReader.getPacket(&pkt, timeSinceStart);
            if (rc == 1) {    // got a full packet?
                if(auto result = decoder.decode(pkt)) {
                    std::pair<velodyne_decoder::Time, velodyne_decoder::PointCloud> pair = result.value();
                    printf("intensity 1st point: %f\n", pair.second[0].intensity); 
                    printf("# points: %d\n", pair.second.size()); 
                    is_scan_full = true;
                }            
            }       
            if (rc < 0) {     // end of file reached?
                printf("end of packet file\n");
                return 0;
            } 
            if (rc == 0) continue;    //timeout?
        }
        printf("curr num packets read : %d\n", i);
    }

return 0;
}
