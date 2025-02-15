//
// Created by consti10 on 11.12.22.
//

#ifndef OPENHD_OPENHD_OHD_INTERFACE_INC_WIFI_CHANNEL_H_
#define OPENHD_OPENHD_OHD_INTERFACE_INC_WIFI_CHANNEL_H_

#include <sstream>
#include <vector>
#include <cstdint>

// NOTE: DO NOT USE CHANNEL NUMBERS ANYWHERE IN CODE - USE FREQUENCIES IN MHZ, SINCE THEY ARE UNIQUE
namespace openhd {

enum class WifiSpace { G2_4, G5_8 };

// Wifi channel and the corresponding frequency, in mHz.
// "standard" : listed under wikipedia or not.
struct WifiChannel {
  // frequency in mhz, recommended to use
  uint32_t frequency;
  // channel corresponding to this frequency, error-prone compared to the frequency since often defined incorrectly and/or for the non-standard AR9271 frequencies there is no standard channel number
  int channel;
  // weather this is a channel in the 2.4G space or 5.8G space
  WifiSpace space;
  // weather this channel is listed under wikipedia or not.
  // Channels not listed under wikipedia might still work on some cards, given the driver has been modified. generally, they are not legally usable in most countries though.
  bool is_standard;
  [[nodiscard]] std::string to_string() const {
    std::stringstream ss;
    ss << (int)frequency << "Mhz [" << channel << "] "
       << (space == WifiSpace::G2_4 ? "2.4G" : "5.8G")
       << (is_standard ? "" : "nonstandard");
    return ss.str();
  }
};

static std::vector<WifiChannel> get_channels_2G() {
  return std::vector<WifiChannel>{
      // These are not valid 2.4G wifi channel(s) but some cards aparently can do them, too From https://github.com/OpenHD/linux/blob/092115ae6a980feaa09722690891d99da3afb55c/drivers/net/wireless/ath/ath9k/common-init.c#L39
      // NOTE: In OpenHD we never use the channel number (since it is prone to errors, even in the linux kernel) but rather use the frequency in mhz, which is well-defined. Also read https://yo3iiu.ro/blog/?p=1301
      WifiChannel{2312, -1, WifiSpace::G2_4, false},
      WifiChannel{2317, -1, WifiSpace::G2_4, false},
      WifiChannel{2322, -1, WifiSpace::G2_4, false},
      WifiChannel{2327, -1, WifiSpace::G2_4, false},
      WifiChannel{2332, -1, WifiSpace::G2_4, false},
      WifiChannel{2337, -1, WifiSpace::G2_4, false},
      WifiChannel{2342, -1, WifiSpace::G2_4, false},
      WifiChannel{2347, -1, WifiSpace::G2_4, false},
      WifiChannel{2352, -1, WifiSpace::G2_4, false},
      WifiChannel{2357, -1, WifiSpace::G2_4, false},
      WifiChannel{2362, -1, WifiSpace::G2_4, false},
      WifiChannel{2367, -1, WifiSpace::G2_4, false},
      WifiChannel{2372, -1, WifiSpace::G2_4, false},
      WifiChannel{2377, -1, WifiSpace::G2_4, false},
      WifiChannel{2382, -1, WifiSpace::G2_4, false},
      WifiChannel{2387, -1, WifiSpace::G2_4, false},
      WifiChannel{2392, -1, WifiSpace::G2_4, false},
      WifiChannel{2397, -1, WifiSpace::G2_4, false},
      WifiChannel{2402, -1, WifiSpace::G2_4, false},
      WifiChannel{2407, -1, WifiSpace::G2_4, false},
      // Now to the standard Wi-Fi channel(s) https://en.wikipedia.org/wiki/List_of_WLAN_channels#2.4_GHz_(802.11b/g/n/ax)
      WifiChannel{2412, 1, WifiSpace::G2_4, true},
      WifiChannel{2417, 2, WifiSpace::G2_4, true},
      WifiChannel{2422, 3, WifiSpace::G2_4, true},
      WifiChannel{2427, 4, WifiSpace::G2_4, true},
      WifiChannel{2432, 5, WifiSpace::G2_4, true},
      WifiChannel{2437, 6, WifiSpace::G2_4, true},
      WifiChannel{2442, 7, WifiSpace::G2_4, true},
      WifiChannel{2447, 8, WifiSpace::G2_4, true},
      WifiChannel{2452, 9, WifiSpace::G2_4, true},
      WifiChannel{2457, 10, WifiSpace::G2_4, true},
      WifiChannel{2462, 11, WifiSpace::G2_4, true},
      WifiChannel{2467, 12, WifiSpace::G2_4, true},
      WifiChannel{2472, 13, WifiSpace::G2_4, true},
      // until here it is consistent (5Mhz increments)
      // this one is neither allowed in EU nor USA
      // (only in Japan under 11b)
      WifiChannel{2484, 14, WifiSpace::G2_4, true},
      // and these are all not valid wlan channels, but the AR9271 can do them anyways
      WifiChannel{2487, -1, WifiSpace::G2_4, false},
      WifiChannel{2489, -1, WifiSpace::G2_4, false},
      WifiChannel{2492, -1, WifiSpace::G2_4, false},
      WifiChannel{2494, -1, WifiSpace::G2_4, false},
      WifiChannel{2497, -1, WifiSpace::G2_4, false},
      WifiChannel{2499, -1, WifiSpace::G2_4, false},
      WifiChannel{2512, -1, WifiSpace::G2_4, false},
      WifiChannel{2532, -1, WifiSpace::G2_4, false},
      WifiChannel{2572, -1, WifiSpace::G2_4, false},
      WifiChannel{2592, -1, WifiSpace::G2_4, false},
      WifiChannel{2612, -1, WifiSpace::G2_4, false},
      WifiChannel{2632, -1, WifiSpace::G2_4, false},
      WifiChannel{2652, -1, WifiSpace::G2_4, false},
      WifiChannel{2672, -1, WifiSpace::G2_4, false},
      WifiChannel{2692, -1, WifiSpace::G2_4, false},
      WifiChannel{2712, -1, WifiSpace::G2_4, false},
      WifiChannel{2732, -1, WifiSpace::G2_4, false},
  };
}


static std::vector<WifiChannel> get_channels_5G(){
  return std::vector<WifiChannel>{
      // https://en.wikipedia.org/wiki/List_of_WLAN_channels#5_GHz_(802.11a/h/j/n/ac/ax)
      WifiChannel{5180, 36, WifiSpace::G5_8, true},
      WifiChannel{5200, 40, WifiSpace::G5_8, true},
      WifiChannel{5220, 44, WifiSpace::G5_8, true},
      WifiChannel{5240, 48, WifiSpace::G5_8, true},
      WifiChannel{5260, 52, WifiSpace::G5_8, true},
      WifiChannel{5280, 56, WifiSpace::G5_8, true},
      WifiChannel{5300, 60, WifiSpace::G5_8, true},
      WifiChannel{5320, 64, WifiSpace::G5_8, true},
      // this part seems to be disabled quite often -beign
      WifiChannel{5340, 68, WifiSpace::G5_8, true},
      WifiChannel{5360, 72, WifiSpace::G5_8, true},
      WifiChannel{5380, 76, WifiSpace::G5_8, true},
      WifiChannel{5400, 80, WifiSpace::G5_8, true},
      WifiChannel{5420, 84, WifiSpace::G5_8, true},
      WifiChannel{5440, 88, WifiSpace::G5_8, true},
      WifiChannel{5460, 92, WifiSpace::G5_8, true},
      WifiChannel{5480, 96, WifiSpace::G5_8, true},
      // part often disabled end
      WifiChannel{5500, 100, WifiSpace::G5_8, true},
      WifiChannel{5520, 104, WifiSpace::G5_8, true},
      WifiChannel{5540, 108, WifiSpace::G5_8, true},
      WifiChannel{5560, 112, WifiSpace::G5_8, true},
      WifiChannel{5580, 116, WifiSpace::G5_8, true},
      WifiChannel{5600, 120, WifiSpace::G5_8, true},
      WifiChannel{5620, 124, WifiSpace::G5_8, true},
      WifiChannel{5640, 128, WifiSpace::G5_8, true},
      WifiChannel{5660, 132, WifiSpace::G5_8, true},
      WifiChannel{5680, 136, WifiSpace::G5_8, true},
      WifiChannel{5700, 140, WifiSpace::G5_8, true},
      WifiChannel{5720, 144, WifiSpace::G5_8, true},
      // For some reason, there is a 25Mhz jump here, at least for rtl8812au
      WifiChannel{5745, 149, WifiSpace::G5_8, true},
      WifiChannel{5765, 153, WifiSpace::G5_8, true},
      WifiChannel{5785, 157, WifiSpace::G5_8, true},
      WifiChannel{5805, 161, WifiSpace::G5_8, true},
      WifiChannel{5825, 165, WifiSpace::G5_8, true},
      // starting from here, often disabled territory begins again
      WifiChannel{5845, 169, WifiSpace::G5_8, true},
      WifiChannel{5865, 173, WifiSpace::G5_8, true},
      WifiChannel{5885, 177, WifiSpace::G5_8, true},
      WifiChannel{5905, 181, WifiSpace::G5_8, true},
  };
};


static std::vector<WifiChannel> get_all_channels_2G_5G() {
  std::vector<WifiChannel> channels{};
  OHDUtil::vec_append(channels, get_channels_2G());
  OHDUtil::vec_append(channels, get_channels_5G());
  return channels;
}

static std::vector<uint32_t> get_all_channel_frequencies(const std::vector<openhd::WifiChannel>& channels) {
  std::vector<uint32_t> frequencies;
  for (const auto& channel : channels) {
    frequencies.push_back(channel.frequency);
  }
  return frequencies;
}

// Returns the corresponding Wi-Fi Channel if there is one
// since the mavlink setting is an int, this might not always be possible (and the frequency is then 100% not possible) and therefore should be discarded / fixed
static std::optional<openhd::WifiChannel> channel_from_frequency(uint32_t frequency) {
  const auto channels = get_all_channels_2G_5G();
  for (const auto& channel : channels) {
    if (channel.frequency == frequency) {
      return channel;
    }
  }
  return std::nullopt;
}

// NOTE: Only call this on a list of frequencies that are valid
static std::vector<openhd::WifiChannel> get_all_channels_from_safe_frequencies(
    const std::vector<uint32_t>& frequencies) {
  std::vector<openhd::WifiChannel> ret;
  for (const auto& freq : frequencies) {
    auto channel= channel_from_frequency(freq);
    assert(channel.has_value());
    ret.push_back(channel.value());
  }
  return ret;
}

static WifiSpace get_space_from_frequency(uint32_t frequency){
  auto channel= channel_from_frequency(frequency);
  if(!channel.has_value()){
    openhd::log::get_default()->warn("Invalid frequency {}, assuming 5G",frequency);
    return WifiSpace::G5_8;
  }
  return channel.value().space;
}

}
#endif  // OPENHD_OPENHD_OHD_INTERFACE_INC_WIFI_CHANNEL_H_
