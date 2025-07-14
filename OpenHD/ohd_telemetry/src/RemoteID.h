//
// Created by rsaxvc on 03.30.25.
//

#ifndef OPENHD_REMOTEID_H
#define OPENHD_REMOTEID_H

#include <memory>
#include <thread>
#include <utility>

#include "endpoints/WBEndpoint.h"
#include "mav_helper.h"
#include "openhd_action_handler.h"
#include "openhd_external_device.h"
#include "openhd_link.hpp"
#include "openhd_link_statistics.hpp"
#include "openhd_platform.h"
#include "openhd_profile.h"
#include "openhd_settings_imp.h"
#include "opendroneid.h"
/**
 * This class holds logic for managing RemoteID.
 */
class RemoteID {
 public:
  RemoteID(OHDProfile profile1, bool enableExtendedLogging = false);
  RemoteID(const RemoteID&) = delete;
  RemoteID(const RemoteID&&) = delete;
  ~RemoteID();
  [[nodiscard]] std::string createDebug() const;
  // RemoteID is agnostic of the type of transmission between air and ground
  // and also agnostic weather this link exists or not (since it is already
  // using a lossy link).
  void set_link_handle(std::shared_ptr<OHDLink> link);
  void process_packet(mavlink_status_t &status, mavlink_message_t &msg);

 private:
  // RemoteID will run in its own threads until terminate is set to true
  // @param enableExtendedLogging be really verbose on logging.
  void loop_infinite(bool& terminate, bool enableExtendedLogging = false);
  void loop_step(uint32_t now_ms);
  void update_send();

  // Main telemetry thread. Note that the endpoints also might have their own
  // Receive threads
  std::unique_ptr<std::thread> m_loop_thread;
  bool m_loop_thread_terminate = false;

  // send/receive data via wb
  std::unique_ptr<WBEndpoint> m_wb_endpoint;

  std::mutex m_components_lock;
  std::shared_ptr<spdlog::logger> m_console;
  const OHDProfile m_profile;
  const bool m_enableExtendedLogging;

  mavlink_channel_t m_mavlinkChan;
  mavlink_system_t m_mavlinkSystem = {0, MAV_COMP_ID_ODID_TXRX_1};

  uint32_t last_location_ms;
  uint32_t last_basic_id_ms;
  uint32_t last_self_id_ms;
  uint32_t last_operator_id_ms;
  uint32_t last_system_ms;
  uint32_t last_system_timestamp;
  float last_location_timestamp;

  mavlink_open_drone_id_location_t location;
  mavlink_open_drone_id_basic_id_t basic_id;
  mavlink_open_drone_id_authentication_t authentication;
  mavlink_open_drone_id_self_id_t self_id;
  mavlink_open_drone_id_system_t system;
  mavlink_open_drone_id_operator_id_t operator_id;

  ODID_UAS_Data UAS_data;
};

#endif  // OPENHD_RemoteID_H
