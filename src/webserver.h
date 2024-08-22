#ifndef WEBSERVER_H
#define WEBSERVER_H
#include <SPI.h>

#include <WiFi.h>

#ifndef HTTP_ANY
#define HTTP_ANY -1
#endif

#include <ESPAsyncWebServer.h>
#ifdef USE_WIFININA
#include <WiFiNINA.h>
#endif
extern double Input;

void setupWiFi();
void setupWebServer();
void loopWebserver();

#endif // WEBSERVER_H

