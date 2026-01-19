#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <unitree/common/time/time_tool.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>

namespace {
constexpr size_t kChunkSize = 96000;
volatile std::sig_atomic_t g_stop_requested = 0;

void HandleSignal(int) {
  g_stop_requested = 1;
}

struct WavInfo {
  uint16_t audio_format = 0;
  uint16_t num_channels = 0;
  uint32_t sample_rate = 0;
  uint16_t bits_per_sample = 0;
  std::vector<uint8_t> pcm;
};

uint8_t ComputeIntensity(const std::vector<uint8_t>& chunk) {
  if (chunk.empty()) {
    return 0;
  }
  double sum_sq = 0.0;
  const size_t sample_count = chunk.size() / sizeof(int16_t);
  const int16_t* samples =
      reinterpret_cast<const int16_t*>(chunk.data());
  for (size_t i = 0; i < sample_count; ++i) {
    const double v = static_cast<double>(samples[i]);
    sum_sq += v * v;
  }
  const double rms = std::sqrt(sum_sq / std::max<size_t>(sample_count, 1));
  const double normalized = std::min(1.0, rms / 32768.0);
  return static_cast<uint8_t>(normalized * 255.0);
}

void LedWorker(unitree::robot::g1::AudioClient* client,
               std::atomic<bool>* stop,
               std::atomic<uint8_t>* intensity) {
  const uint8_t palette[][3] = {
      {255, 0, 0},    // red
      {255, 128, 0},  // orange
      {255, 255, 0},  // yellow
      {0, 255, 0},    // green
      {0, 255, 255},  // cyan
      {0, 0, 255},    // blue
      {255, 0, 255}   // magenta
  };
  size_t palette_index = 0;
  const int kFadeSteps = 40;
  int step = 0;
  double smooth_intensity = 0.0;
  while (!stop->load()) {
    uint8_t target = intensity->load();
    if (target > smooth_intensity) {
      smooth_intensity = smooth_intensity * 0.6 + target * 0.4;
    } else {
      smooth_intensity = smooth_intensity * 0.9 + target * 0.1;
    }
    const size_t next_index =
        (palette_index + 1) % (sizeof(palette) / sizeof(palette[0]));
    const double t = static_cast<double>(step) / kFadeSteps;
    const double r_base =
        palette[palette_index][0] * (1.0 - t) + palette[next_index][0] * t;
    const double g_base =
        palette[palette_index][1] * (1.0 - t) + palette[next_index][1] * t;
    const double b_base =
        palette[palette_index][2] * (1.0 - t) + palette[next_index][2] * t;
    const uint8_t r =
        static_cast<uint8_t>(r_base * smooth_intensity / 255.0);
    const uint8_t g =
        static_cast<uint8_t>(g_base * smooth_intensity / 255.0);
    const uint8_t b =
        static_cast<uint8_t>(b_base * smooth_intensity / 255.0);
    client->LedControl(r, g, b);
    step = (step + 1) % kFadeSteps;
    if (step == 0) {
      palette_index = next_index;
    }
    unitree::common::MilliSleep(50);
  }
}

bool ReadWavFile(const std::string& path, WavInfo* info) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    std::cout << "Failed to open wav file: " << path << std::endl;
    return false;
  }

  char riff[4] = {0};
  uint32_t riff_size = 0;
  char wave[4] = {0};
  in.read(riff, 4);
  in.read(reinterpret_cast<char*>(&riff_size), sizeof(riff_size));
  in.read(wave, 4);
  if (std::string(riff, 4) != "RIFF" || std::string(wave, 4) != "WAVE") {
    std::cout << "Invalid WAV header." << std::endl;
    return false;
  }

  bool fmt_found = false;
  bool data_found = false;
  while (in && (!fmt_found || !data_found)) {
    char chunk_id[4] = {0};
    uint32_t chunk_size = 0;
    in.read(chunk_id, 4);
    in.read(reinterpret_cast<char*>(&chunk_size), sizeof(chunk_size));
    if (!in) {
      break;
    }

    std::string id(chunk_id, 4);
    if (id == "fmt ") {
      fmt_found = true;
      in.read(reinterpret_cast<char*>(&info->audio_format),
              sizeof(info->audio_format));
      in.read(reinterpret_cast<char*>(&info->num_channels),
              sizeof(info->num_channels));
      in.read(reinterpret_cast<char*>(&info->sample_rate),
              sizeof(info->sample_rate));
      uint32_t byte_rate = 0;
      uint16_t block_align = 0;
      in.read(reinterpret_cast<char*>(&byte_rate), sizeof(byte_rate));
      in.read(reinterpret_cast<char*>(&block_align), sizeof(block_align));
      in.read(reinterpret_cast<char*>(&info->bits_per_sample),
              sizeof(info->bits_per_sample));

      if (chunk_size > 16) {
        in.ignore(chunk_size - 16);
      }
    } else if (id == "data") {
      data_found = true;
      info->pcm.resize(chunk_size);
      in.read(reinterpret_cast<char*>(info->pcm.data()), chunk_size);
    } else {
      in.ignore(chunk_size);
    }
  }

  if (!fmt_found || !data_found) {
    std::cout << "Missing fmt/data chunks in WAV file." << std::endl;
    return false;
  }

  return true;
}
}  // namespace

int main(int argc, char const* argv[]) {
  if (argc < 3) {
    std::cout << "Usage: g1_audio_play_test [NetWorkInterface(eth0)] [wav_path]"
              << std::endl;
    return 1;
  }

  std::string wav_path = argv[2];

  WavInfo info;
  if (!ReadWavFile(wav_path, &info)) {
    return 1;
  }

  std::cout << "wav file sample_rate=" << info.sample_rate
            << " num_channels=" << info.num_channels
            << " bits_per_sample=" << info.bits_per_sample
            << " size=" << info.pcm.size() << std::endl;

  if (info.audio_format != 1 || info.sample_rate != 16000 ||
      info.num_channels != 1 || info.bits_per_sample != 16) {
    std::cout << "Unsupported WAV format. Need PCM 16kHz mono 16-bit."
              << std::endl;
    return 1;
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::g1::AudioClient client;
  client.Init();
  client.SetTimeout(10.0f);

  std::string stream_id =
      std::to_string(unitree::common::GetCurrentTimeMillisecond());
  std::signal(SIGINT, HandleSignal);

  std::atomic<bool> led_stop(false);
  std::atomic<uint8_t> intensity(0);
  std::thread led_thread(LedWorker, &client, &led_stop, &intensity);

  size_t offset = 0;
  while (offset < info.pcm.size()) {
    if (g_stop_requested != 0) {
      break;
    }
    size_t remaining = info.pcm.size() - offset;
    size_t current_chunk_size = std::min(kChunkSize, remaining);
    std::vector<uint8_t> chunk(info.pcm.begin() + offset,
                               info.pcm.begin() + offset + current_chunk_size);
    int32_t ret = client.PlayStream("play_test", stream_id, chunk);
    std::cout << "PlayStream ret: " << ret << " offset=" << offset << std::endl;

    intensity.store(ComputeIntensity(chunk));
    unitree::common::Sleep(1);
    offset += current_chunk_size;
  }

  client.PlayStop(stream_id);
  led_stop.store(true);
  led_thread.join();
  if (g_stop_requested == 0) {
    std::cout << "Playback complete. Running LED sequence..." << std::endl;
    int32_t led_ret = client.LedControl(0, 255, 0);
    std::cout << "LedControl green ret: " << led_ret << std::endl;
    unitree::common::Sleep(1);
    led_ret = client.LedControl(0, 0, 255);
    std::cout << "LedControl blue ret: " << led_ret << std::endl;
    unitree::common::Sleep(1);
    led_ret = client.LedControl(0, 0, 0);
    std::cout << "LedControl off ret: " << led_ret << std::endl;
  }
  client.LedControl(0, 0, 0);
  return 0;
}
