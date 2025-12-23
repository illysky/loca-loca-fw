#ifndef WIFI_H
#define WIFI_H

#include "esp_err.h"

/**
 * @brief Initialize WiFi
 * @return ESP_OK on success
 */
esp_err_t wifi_init(void);

/**
 * @brief Connect to WiFi network
 * @param ssid WiFi SSID
 * @param password WiFi password
 * @return ESP_OK on success
 */
esp_err_t wifi_connect(const char *ssid, const char *password);

/**
 * @brief Check if WiFi is connected
 * @return true if connected
 */
bool wifi_is_connected(void);

/**
 * @brief Get current IP address
 * @param ip_str Buffer to store IP string (must be at least 16 bytes)
 * @return ESP_OK on success
 */
esp_err_t wifi_get_ip(char *ip_str);

#endif // WIFI_H

