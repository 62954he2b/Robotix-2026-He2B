#ifndef ACCESSPOINTESP_H
#define ACCESSPOINTESP_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "cli_input.h"

extern const char* ssid;
extern const char* password;

extern bool page_loaded;
extern String input_message;

void initiate_access_point(const char* ssid, const char* password);
void read_wifi_input_task(void *parameter);

#endif