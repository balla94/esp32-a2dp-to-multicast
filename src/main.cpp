#include <Arduino.h>
/**
 * @file basic-a2dp-audioi2s.ino
 * @brief A2DP Sink with output to I2SStream. This example is of small value
 * since my Bluetooth Library already provides I2S output out of the box.
 *
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

#include "AudioTools.h"
#include <ETH.h>
#include <esp_eth.h>
#include <ArduinoOTA.h>
#include "Communication/UDPStream.h"
#include <BluetoothA2DPSinkQueued.h>

#define UDP_CHUNK_SIZE 1400

BluetoothA2DPSinkQueued a2dp_sink;
UDPStream udpStream;

void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);

  switch (event)
  {
  case ARDUINO_EVENT_WIFI_READY:
    Serial.println("WiFi interface ready");
    break;
  case ARDUINO_EVENT_WIFI_SCAN_DONE:
    Serial.println("Completed scan for access points");
    break;
  case ARDUINO_EVENT_WIFI_STA_START:
    Serial.println("WiFi client started");
    break;
  case ARDUINO_EVENT_WIFI_STA_STOP:
    Serial.println("WiFi clients stopped");
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    Serial.println("Connected to access point");
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("Disconnected from WiFi access point");
    break;
  case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
    Serial.println("Authentication mode of access point has changed");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.print("Obtained IP address: ");
    Serial.println(WiFi.localIP());
    break;
  case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    Serial.println("Lost IP address and IP address is reset to 0");
    break;
  case ARDUINO_EVENT_WPS_ER_SUCCESS:
    Serial.println("WiFi Protected Setup (WPS): succeeded in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_FAILED:
    Serial.println("WiFi Protected Setup (WPS): failed in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_TIMEOUT:
    Serial.println("WiFi Protected Setup (WPS): timeout in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_PIN:
    Serial.println("WiFi Protected Setup (WPS): pin code in enrollee mode");
    break;
  case ARDUINO_EVENT_WIFI_AP_START:
    Serial.println("WiFi access point started");
    break;
  case ARDUINO_EVENT_WIFI_AP_STOP:
    Serial.println("WiFi access point  stopped");
    break;
  case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    Serial.println("Client connected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    Serial.println("Client disconnected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
    Serial.println("Assigned IP address to client");
    break;
  case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
    Serial.println("Received probe request");
    break;
  case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
    Serial.println("AP IPv6 is preferred");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    Serial.println("STA IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP6:
    Serial.println("Ethernet IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_START:
    Serial.println("Ethernet started");
    break;
  case ARDUINO_EVENT_ETH_STOP:
    Serial.println("Ethernet stopped");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    Serial.println("Ethernet connected");
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    Serial.println("Ethernet disconnected");
    udpStream.end();
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    Serial.println("Obtained IP address");
    Serial.println(ETH.localIP());
    udpStream.begin(IPAddress(239, 0, 1, 118), 8077);

    break;
  default:
    break;
  }
}

String convertMillisToReadableTime(unsigned long millis)
{
  unsigned long seconds = millis / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;

  unsigned long displayMillis = millis % 1000;
  unsigned long displaySeconds = seconds % 60;
  unsigned long displayMinutes = minutes % 60;
  unsigned long displayHours = hours;

  // Create a human-readable time string
  char timeString[50];
  snprintf(timeString, sizeof(timeString), "%02lu:%02lu:%02lu.%03lu", displayHours, displayMinutes, displaySeconds, displayMillis);

  return String(timeString);
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  uint16_t written;
#ifdef ESP32_GATEWAY
  digitalWrite(LED_BUILTIN, HIGH);
#endif
  written = udpStream.write(data, length);
#ifdef ESP32_GATEWAY
  digitalWrite(LED_BUILTIN, LOW);
#endif
  if (written != length)
  {
    Serial.printf("UDP error, expected: %d, actual: %d\n", length, written);
  }
}


void avrc_metadata_callback(uint8_t id, const uint8_t *text)
{
  Serial.printf("==> AVRC metadata rsp: attribute id 0x%x, %s\n", id, text);
  if (id == ESP_AVRC_MD_ATTR_PLAYING_TIME)
  {
    uint32_t playtime = String((char *)text).toInt();
    Serial.printf("==> Playing time is %d ms %d, ", playtime);
    Serial.println(convertMillisToReadableTime(playtime));
  }
}
void avrc_connection_callback(bool state)
{
  Serial.print("Connection state is now: ");
  Serial.println(state ? "true" : "false");
  if (state)
  {
    a2dp_sink.set_volume(100);
  }
}
void avrc_volume_callback(int level)
{
  Serial.print("Volume is now: ");
  Serial.println(level);
}

void avrc_play_pos(uint32_t position)
{
  Serial.print("Play position: ");
  Serial.println(convertMillisToReadableTime(position));
}

void avrc_play_state(esp_avrc_playback_stat_t playback)
{
  Serial.print("Play status: ");
  Serial.println(playback);
}

void setup()
{
  delay(2000);
  Serial.begin(115200);
#ifdef ESP32_GATEWAY
  pinMode(LED_BUILTIN, OUTPUT);
#endif
  // AudioLogger::instance().begin(Serial, AudioLogger::Info);
  WiFi.onEvent(WiFiEvent);
  ETH.begin();
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.disconnect();

  // setup output
  udpStream.begin(IPAddress(239, 0, 1, 118), 8077);

  // register callback
  a2dp_sink.set_stream_reader(read_data_stream, false);

  // Start Bluetooth Audio Receiver
  a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.set_avrc_metadata_attribute_mask(0xFF);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_avrc_connection_state_callback(avrc_connection_callback);
  a2dp_sink.set_avrc_rn_volumechange(avrc_volume_callback);
  a2dp_sink.set_avrc_rn_volumechange_completed(avrc_volume_callback);
  a2dp_sink.set_avrc_rn_play_pos_callback(avrc_play_pos, 1);
  a2dp_sink.set_avrc_rn_playstatus_callback(avrc_play_state);
  a2dp_sink.set_i2s_ringbuffer_prefetch_percent(90);
  a2dp_sink.start("SzarvasA2DP");
}
void loop()
{
  delay(1);
}