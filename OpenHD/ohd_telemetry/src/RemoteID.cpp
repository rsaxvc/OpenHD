//
// Created by rsaxvc on 03.30.25.
// Based heavily on ArduRemoteID - ESP32
//

#include "openhd_util.h"
#include "openhd_util_time.h"

#include "RemoteID.h"

#include <common/mavlink.h>
#include <openhd/mavlink.h>

static uint32_t millis(void){
  return openhd::util::steady_clock_time_epoch_ms();
}

RemoteID::RemoteID(OHDProfile profile1, bool enableExtendedLogging)
    : m_profile(std::move(profile1)),
      m_enableExtendedLogging(enableExtendedLogging) {
  m_console = openhd::log::create_or_get("RemoteID");
  m_loop_thread = std::make_unique<std::thread>([this] {
    loop_infinite(m_loop_thread_terminate, this->m_enableExtendedLogging);
  });
}

RemoteID::~RemoteID() {
  m_loop_thread_terminate = true;
  m_loop_thread->join();
}

std::string RemoteID::createDebug() const {
  return "noDebug";
}

void RemoteID::set_link_handle(std::shared_ptr<OHDLink> link) {
  // only call this once, we do not support changing the link handle at run time
  if (link == nullptr) {
    openhd::log::get_default()->warn("set_link_handle - no link available");
    return;
  }
  assert(m_wb_endpoint == nullptr);
  m_wb_endpoint = std::make_unique<WBEndpoint>(link, "remoteid_tx");
#if 0
  m_wb_endpoint->registerCallback([this](std::vector<MavlinkMessage> messages) {
    on_messages_air_unit(messages);
  });
#endif
}

void RemoteID::loop_infinite(bool& terminate, bool enableExtendedLogging) {
  const auto log_interval = std::chrono::seconds(5);
  const auto loop_interval = std::chrono::seconds(1);
  const auto wifi_tu = std::chrono::microseconds(1024);
  auto last_log = std::chrono::steady_clock::now();
  while (!terminate) {
    const uint32_t now_ms = millis();
    const auto loopBegin = std::chrono::steady_clock::now();
    if (std::chrono::steady_clock::now() - last_log >= log_interval) {
      last_log = std::chrono::steady_clock::now();
      // m_console->debug("RemoteID::loopInfinite()");
      //  for debugging, check if any of the endpoints is not alive
      if (enableExtendedLogging && m_wb_endpoint) {
        m_console->debug(m_wb_endpoint->createInfo());
      }
    }
    // send Heartbeats and such.
    update_send();

    // send RemoteID broadcast beacons in regular intervals.
    // everything else is handled by the callbacks and their threads
    {
      std::lock_guard<std::mutex> guard(m_components_lock);

      if (last_location_ms == 0 ||
        now_ms - last_location_ms > 5000) {
        UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
      }
      if (last_system_ms == 0 ||
        now_ms - last_system_ms > 5000) {
        UAS_data.Location.Status = ODID_STATUS_REMOTE_ID_SYSTEM_FAILURE;
      }


      // TODO: use proper values here
      char mac[6] = {0x11,0x22,0x33,0x44,0x55,0x66};
      char SSID[] = "RSAXVC";
      uint8_t buf[2048];
      uint16_t interval_tu = loop_interval / wifi_tu;
      uint8_t send_counter = 3;
      int odid_rc = odid_wifi_build_message_pack_beacon_frame(&UAS_data, mac, SSID, strlen(SSID),
                                              interval_tu, send_counter, buf, sizeof(buf));
      if(odid_rc < 0) {
        m_console->error("failed to encode beacon frame, rc:%i", odid_rc);
      }
      else if(odid_rc > 0) {
        // RSAXVC TODO: send RemoteID packet over WiFi Beacon
        if(m_wb_endpoint) {

          std::shared_ptr<std::vector<uint8_t>> packet;
          RemoteIdPacket remoteIdPacket;
          remoteIdPacket.data = packet;
          m_wb_endpoint->sendRemoteId(remoteIdPacket);
        }
      }
    }
    const auto loopDelta = std::chrono::steady_clock::now() - loopBegin;
    if (loopDelta > loop_interval) {
      // We can't keep up with the wanted loop interval
      m_console->debug(
          "Warning RemoteID cannot keep up with the wanted loop "
          "interval. Took {}",
          openhd::util::time_readable(loopDelta));
    } else {
      const auto sleepTime = loop_interval - loopDelta;
      // send out in X second intervals
      std::this_thread::sleep_for(loop_interval);
    }
  }
}

void RemoteID::loop_step(uint32_t now_ms)
{
    static uint32_t last_hb_warn_ms = 0;

    if (m_mavlinkSystem.sysid != 0) {
        update_send();
//    } else if (g.mavlink_sysid != 0) {
//        m_mavlinkSystem.sysid = g.mavlink_sysid;
    } else if (now_ms - last_hb_warn_ms >= 2000) {
        last_hb_warn_ms = now_ms;
        m_console->info("Waiting for heartbeat");
    }
}

void RemoteID::update_send(void)
{
    //TODO: how do we send these back to ground station?
#if 0
    // send heartbeat - could this be combined with VTX
    mavlink_msg_heartbeat_send(
        m_mavlinkChan,
        MAV_TYPE_ODID,
        MAV_AUTOPILOT_INVALID,
        0,
        0,
        0);

    // send arming status
    mavlink_msg_open_drone_id_arm_status_send(m_mavlinkChan, 0, "developing");
#endif
}


void RemoteID::process_packet(mavlink_status_t &status, mavlink_message_t &msg)
{
    const uint32_t now_ms = millis();
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        m_console->debug("got Heartbeat");
        mavlink_heartbeat_t hb;
        if (m_mavlinkSystem.sysid == 0) {
            mavlink_msg_heartbeat_decode(&msg, &hb);
            if (msg.sysid > 0 && hb.type != MAV_TYPE_GCS) {
                m_mavlinkSystem.sysid = msg.sysid;
            }
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_LOCATION: {
        m_console->info("got Location");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_location_decode(&msg, &location);
        if (last_location_timestamp != location.timestamp) {
            //only update the timestamp if we receive information with a different timestamp
            last_location_ms = now_ms;
            last_location_timestamp = location.timestamp;
        }
        last_location_ms = now_ms;
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_BASIC_ID: {
        mavlink_open_drone_id_basic_id_t basic_id_tmp;
        m_console->info("got BasicID");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_basic_id_decode(&msg, &basic_id_tmp);
        if ((strlen((const char*) basic_id_tmp.uas_id) > 0) && (basic_id_tmp.id_type > 0) && (basic_id_tmp.id_type <= MAV_ODID_ID_TYPE_SPECIFIC_SESSION_ID)) {
            //only update if we receive valid data
            basic_id = basic_id_tmp;
            last_basic_id_ms = now_ms;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_AUTHENTICATION: {
        m_console->info("got Auth");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_authentication_decode(&msg, &authentication);
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID: {
        m_console->info("got SelfID");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_self_id_decode(&msg, &self_id);
        last_self_id_ms = now_ms;
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM: {
        m_console->info("got System");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_system_decode(&msg, &system);
        if ((last_system_timestamp != system.timestamp) || (system.timestamp == 0)) {
            //only update the timestamp if we receive information with a different timestamp
            last_system_ms = now_ms;
            last_system_timestamp = system.timestamp;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_SYSTEM_UPDATE: {
        m_console->info("got System update");
        mavlink_open_drone_id_system_update_t pkt_system_update = {};
        mavlink_msg_open_drone_id_system_update_decode(&msg, &pkt_system_update);
        std::lock_guard<std::mutex> guard(m_components_lock);
        //m_mavlinkSystem.operator_latitude = pkt_system_update.operator_latitude;
        //m_mavlinkSystem.operator_longitude = pkt_system_update.operator_longitude;
        //m_mavlinkSystem.operator_altitude_geo = pkt_system_update.operator_altitude_geo;
        //m_mavlinkSystem.timestamp = pkt_system_update.timestamp;
        if (last_system_ms != 0) {
            // we can only mark system as updated if we have the other
            // information already
            if ((last_system_timestamp != system.timestamp) || (pkt_system_update.timestamp == 0)) {
                //only update the timestamp if we receive information with a different timestamp
                last_system_timestamp = pkt_system_update.timestamp;
            }
            last_system_ms = now_ms;
        }
        break;
    }
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_OPERATOR_ID: {
        m_console->info("got OperatorID");
        std::lock_guard<std::mutex> guard(m_components_lock);
        mavlink_msg_open_drone_id_operator_id_decode(&msg, &operator_id);
        last_operator_id_ms = now_ms;
        break;
    }
    default:
        // we don't care about other packets
        break;
    }
}
