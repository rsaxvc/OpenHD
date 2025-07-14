//
// Created by rsaxvc on 31/08/2025
//

#ifndef OPENHD_OPENHD_OHD_COMMON_OPENHD_REMOTEID_H_
#define OPENHD_OPENHD_OHD_COMMON_OPENHD_REMOTEID_H_

#include <cstdint>
#include <memory>
#include <vector>

// Header serves as common declaration for RemoteIdPacket

struct RemoteIdPacket {
  std::shared_ptr<std::vector<uint8_t>> data;
};

#endif  // OPENHD_OPENHD_OHD_COMMON_OPENHD_REMOTEID_H_
