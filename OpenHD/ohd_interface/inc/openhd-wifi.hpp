#ifndef OPENHD_WIFI_H
#define OPENHD_WIFI_H

#include <string>
#include <fstream>

#include "json.hpp"
#include "openhd-util.hpp"
#include "openhd-log.hpp"
#include "openhd-settings.hpp"
#include "openhd-util-filesystem.hpp"
#include "openhd-settings2.hpp"
#include "validate_settings_helper.h"
#include "mavlink_settings/ISettingsComponent.h"

// R.n (20.08) this class can be summarized as following:
// 1) WifiCard: Capabilities of a detected wifi card, no persistent settings
// 2) WifiCardSettings: What to use the wifi card for, !! no frequency settings or similar. !! (note that freq and more are figured out /stored
// by the different interface types that use wifi, e.g. WBStreams or WifiHotspot

enum class WiFiCardType {
  Unknown = 0,
  Realtek8812au,
  Realtek8814au,
  Realtek88x2bu,
  Realtek8188eu,
  Atheros9khtc,
  Atheros9k,
  Ralink,
  Intel,
  Broadcom,
};
NLOHMANN_JSON_SERIALIZE_ENUM( WiFiCardType, {
   {WiFiCardType::Unknown, nullptr},
   {WiFiCardType::Realtek8812au, "Realtek8812au"},
   {WiFiCardType::Realtek8814au, "Realtek8814au"},
   {WiFiCardType::Realtek88x2bu, "Realtek88x2bu"},
   {WiFiCardType::Realtek8188eu, "Realtek8188eu"},
   {WiFiCardType::Atheros9khtc, "Atheros9khtc"},
   {WiFiCardType::Atheros9k, "Atheros9k"},
   {WiFiCardType::Ralink, "Ralink"},
   {WiFiCardType::Intel, "Intel"},
   {WiFiCardType::Broadcom, "Broadcom"},
});

static std::string wifi_card_type_to_string(const WiFiCardType &card_type) {
  switch (card_type) {
	case WiFiCardType::Realtek8812au:return "Realtek8812au";
	case WiFiCardType::Realtek8814au:return  "Realtek8814au";
	case WiFiCardType::Realtek88x2bu:return  "Realtek88x2bu";
	case WiFiCardType::Realtek8188eu:return  "Realtek8188eu";
	case WiFiCardType::Atheros9khtc:return  "Atheros9khtc";
	case WiFiCardType::Atheros9k:return  "Atheros9k";
	case WiFiCardType::Ralink:return  "Ralink";
	case WiFiCardType::Intel:return  "Intel";
	case WiFiCardType::Broadcom:return  "Broadcom";
	default: return "unknown";
  }
}

enum class WiFiHotspotType {
  None = 0,
  Internal2GBand,
  Internal5GBand,
  InternalDualBand,
  External,
};
static std::string wifi_hotspot_type_to_string(const WiFiHotspotType &wifi_hotspot_type) {
  switch (wifi_hotspot_type) {
    case WiFiHotspotType::Internal2GBand:return "internal2g";
    case WiFiHotspotType::Internal5GBand:  return "internal5g";
    case WiFiHotspotType::InternalDualBand:  return "internaldualband";
    case WiFiHotspotType::External:  return "external";
    case WiFiHotspotType::None:
    default:
      return "none";
  }
}

// What to use a discovered wifi card for. R.n We support hotspot or monitor mode (wifibroadcast),
// I doubt that will change.
enum class WifiUseFor {
  Unknown = 0, // Not sure what to use this wifi card for, aka unused.
  MonitorMode, //Use for wifibroadcast, aka set to monitor mode.
  Hotspot, //Use for hotspot, aka start a wifi hotspot with it.
};
NLOHMANN_JSON_SERIALIZE_ENUM( WifiUseFor, {
   {WifiUseFor::Unknown, nullptr},
   {WifiUseFor::MonitorMode, "MonitorMode"},
   {WifiUseFor::Hotspot, "Hotspot"},
});
static std::string wifi_use_for_to_string(const WifiUseFor wifi_use_for){
  switch (wifi_use_for) {
    case WifiUseFor::Hotspot:return "hotspot";
    case WifiUseFor::MonitorMode:return "monitor_mode";
    case WifiUseFor::Unknown:
    default:
      return "unknown";
  }
}

// NOTE: Frequency is not a "card" param, it is set in the WB stream settings for WB and
// In the hotpspot settings for wifi hotspot.
struct WifiCardSettings{
  // This one needs to be set for the card to then be used for something.Otherwise, it is not used for anything
  WifiUseFor use_for = WifiUseFor::Unknown;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WifiCardSettings,use_for)//,frequency,txpower)

struct WiFiCard {
  std::string driver_name; // Name of the driver that runs this card.
  WiFiCardType type = WiFiCardType::Unknown; // Detected wifi card type, generated by checking known drivers.
  std::string interface_name;
  std::string mac;
  bool supports_5ghz = false;
  bool supports_2ghz = false;
  bool supports_injection = false;
  bool supports_hotspot = false;
  bool supports_rts = false;
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WiFiCard,driver_name,type,interface_name,mac,supports_5ghz,supports_2ghz,
                                   supports_injection,supports_hotspot,supports_rts)

static WifiCardSettings create_default_settings(const WiFiCard& wifi_card){
  WifiCardSettings settings;
  if(wifi_card.supports_injection){
    settings.use_for=WifiUseFor::MonitorMode;
  }else if(wifi_card.supports_hotspot){
    // cannot do monitor mode, so we do hotspot with it
    settings.use_for=WifiUseFor::Hotspot;
  }else{
    settings.use_for=WifiUseFor::Unknown;
  }
  // if a card is not functional, Discovery should not make it available.
  // ( a card either has to do 2.4 or 5 ghz, otherise - what the heck ;)
  assert(wifi_card.supports_5ghz || wifi_card.supports_2ghz);
  return settings;
}

static const std::string WIFI_SETTINGS_DIRECTORY=std::string(BASE_PATH)+std::string("interface/");
// WifiCardHolder is used to
// 1) Differentiate between immutable information (like mac address) and
// 2) mutable WiFi card settings.
// Setting changes are propagated through this class.
 class WifiCardHolder : public openhd::settings::PersistentSettings<WifiCardSettings>{
 public:
  explicit WifiCardHolder(WiFiCard wifi_card):
	  openhd::settings::PersistentSettings<WifiCardSettings>(WIFI_SETTINGS_DIRECTORY),
	  _wifi_card(std::move(wifi_card)){
    init();
  }
 public:
  const WiFiCard _wifi_card;
 private:
   [[nodiscard]] std::string get_unique_filename()const override{
    std::stringstream ss;
    ss<<wifi_card_type_to_string(_wifi_card.type)<<"_"<<_wifi_card.interface_name;
    return ss.str();
  }
   [[nodiscard]] WifiCardSettings create_default()const override{
	 return create_default_settings(_wifi_card);
   }
};


static nlohmann::json wificards_to_json(const std::vector<WiFiCard> &cards) {
  nlohmann::json j;
  for (auto &_card: cards) {
	nlohmann::json cardJson = _card;
	j.push_back(cardJson);
  }
  return j;
}

static constexpr auto WIFI_MANIFEST_FILENAME = "/tmp/wifi_manifest";

static void write_wificards_manifest(const std::vector<WiFiCard> &cards) {
  auto manifest = wificards_to_json(cards);
  std::ofstream _t(WIFI_MANIFEST_FILENAME);
  _t << manifest.dump(4);
  _t.close();
}


#endif
