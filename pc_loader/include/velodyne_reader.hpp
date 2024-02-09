#pragma once

#include <string>
#include <cstdlib>
#include "velodyne_decoder/input.h"
#include "velodyne_decoder/packet_decoder.h"
#include "velodyne_decoder/config.h"
#include "velodyne_decoder/stream_decoder.h"
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <memory>

using namespace std;
using namespace velodyne_decoder;

class VelodyneReaderAdapter {

    public:
        VelodyneReaderAdapter(string filename, string calibration_file) :
            packetReader(velodyne_decoder::DATA_PORT_NUMBER, 10.0, filename, true),
            config{"VLP-16", calibration_file} {
            
            // 600 / 60 = 10 frames per second (Hz) at which the packets are collected
            config.rpm = 600;
            decoder_ptr = std::make_shared<velodyne_decoder::StreamDecoder>(config);
        };

        void read_scan(velodyne_decoder::PointCloud&);


    private:
        velodyne_decoder::InputPCAP packetReader;
        velodyne_decoder::Config config;
        std::shared_ptr<velodyne_decoder::StreamDecoder> decoder_ptr = nullptr;
        double timeSinceStart = 0;
};