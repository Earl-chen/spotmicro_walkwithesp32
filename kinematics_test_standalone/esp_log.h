// ESP-IDF compatibility layer for standard C++
#pragma once

#include <iostream>
#include <cstdio>

// ESP Log levels
typedef enum {
    ESP_LOG_ERROR,
    ESP_LOG_WARN,
    ESP_LOG_INFO,
    ESP_LOG_DEBUG,
    ESP_LOG_VERBOSE
} esp_log_level_t;

// Mock ESP_LOG macros
#define ESP_LOGE(tag, format, ...) printf("[ERROR] %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) printf("[WARN] %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) printf("[INFO] %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) printf("[DEBUG] %s: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGV(tag, format, ...) printf("[VERBOSE] %s: " format "\n", tag, ##__VA_ARGS__)