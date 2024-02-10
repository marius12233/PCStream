#include "velodyne_reader.hpp"

void VelodyneReaderAdapter::read_scan(velodyne_decoder::PointCloud& point_cloud) {
    bool is_scan_full = false;
    while (!is_scan_full) {
        VelodynePacket pkt;
        // keep reading until full packet received
        int rc = packetReader.getPacket(&pkt, timeSinceStart);
        if (rc == 1) {    // got a full packet?
            if(auto result = decoder_ptr->decode(pkt)) {
                std::pair<velodyne_decoder::Time, velodyne_decoder::PointCloud> pair = result.value();
                is_scan_full = true;
                point_cloud = pair.second;
            }            
        }       
        if (rc < 0) {     // end of file reached?
            printf("end of packet file\n");
            return;
        } 
        if (rc == 0) continue;    //timeout?
    }
}