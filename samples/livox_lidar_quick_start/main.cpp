//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//

#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

#ifdef _WIN32
#include <winsock2.h>
#else
#include <unistd.h>
#include <arpa/inet.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>

// Global variables
std::map<uint32_t, std::string> g_lidar_map;
std::map<uint32_t, uint8_t> g_lidar_state_map;
std::mutex g_lidar_mutex;
bool g_mode_changed = false;

void WorkModeCallback(livox_status status, uint32_t handle, LivoxLidarAsyncControlResponse *response, void *client_data) {
  if (response == nullptr) return;
  
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;
  
  if (status == kLivoxLidarStatusSuccess && response->ret_code == 0) {
    printf("✓ Lidar %s switched\n", inet_ntoa(tmp_addr));
    if (client_data != nullptr) {
      uint8_t* mode = (uint8_t*)client_data;
      std::lock_guard<std::mutex> lock(g_lidar_mutex);
      g_lidar_state_map[handle] = *mode;
      g_mode_changed = true;
    }
  } else {
    printf("✗ Lidar %s switch failed\n", inet_ntoa(tmp_addr));
  }
}

void LidarInfoChangeCallback(const uint32_t handle, const LivoxLidarInfo* info, void* client_data) {
  if (info == nullptr) return;
  
  struct in_addr tmp_addr;
  tmp_addr.s_addr = handle;
  
  {
    std::lock_guard<std::mutex> lock(g_lidar_mutex);
    g_lidar_map[handle] = std::string(info->sn);
    g_lidar_state_map[handle] = 0x02;  // Assume STANDBY initially
  }
  
  printf("✓ Lidar found: %s (SN: %s)\n", inet_ntoa(tmp_addr), info->sn);
}

int main(int argc, const char *argv[]) {
  if (argc != 3) {
    printf("Usage: %s <config.json> <on|off>\n", argv[0]);
    return -1;
  }

  std::string mode_arg = argv[2];
  bool turn_on;
  
  if (mode_arg == "on") {
    turn_on = true;
  } else if (mode_arg == "off") {
    turn_on = false;
  } else {
    printf("Invalid mode. Use 'on' or 'off'\n");
    return -1;
  }

  if (!LivoxLidarSdkInit(argv[1])) {
    printf("Init failed\n");
    return -1;
  }
  
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallback, nullptr);

  printf("\n╔════════════════════════════════════════╗\n");
  printf("║  Livox Lidar Control                  ║\n");
  printf("╚════════════════════════════════════════╝\n");
  printf("\nScanning for lidars...\n");

  // Wait for lidars to connect
  std::this_thread::sleep_for(std::chrono::seconds(3));

  std::lock_guard<std::mutex> lock(g_lidar_mutex);
  
  if (g_lidar_map.empty()) {
    printf("\n✗ No lidars found\n");
    LivoxLidarSdkUninit();
    return -1;
  }

  LivoxLidarWorkMode target_mode;
  static uint8_t mode_value;
  
  if (turn_on) {
    target_mode = kLivoxLidarNormal;
    mode_value = 0x01;
    printf("\nTurning lidars 🟢 ON...\n\n");
  } else {
    target_mode = kLivoxLidarWakeUp;
    mode_value = 0x02;
    printf("\nTurning lidars 🟡 OFF (STANDBY)...\n\n");
  }
  
  // Set mode for all lidars
  for (const auto& pair : g_lidar_map) {
    SetLivoxLidarWorkMode(pair.first, target_mode, WorkModeCallback, &mode_value);
  }
  
  // Wait for completion
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  printf("\nDone!\n");
  
  LivoxLidarSdkUninit();
  return 0;
}
