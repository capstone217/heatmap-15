#include <WiFi.h>

const unsigned long WIFI_SCAN_INTERVAL_MS = 3000;
unsigned long lastWifiScanMs = 0;

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(200);
}

void loop() {
  if (millis() - lastWifiScanMs >= WIFI_SCAN_INTERVAL_MS) {
    lastWifiScanMs = millis();
    scanWifi24GHz();
  }

  // Keep your existing UWB ranging code here.
  // It can continue printing POS,NAN,NAN,d1,d2,d3,d4,...
}

void scanWifi24GHz() {
  Serial.println("WIFI_SCAN_START");

  int count = WiFi.scanNetworks(false, true);
  for (int i = 0; i < count; i++) {
    int channel = WiFi.channel(i);
    if (channel < 1 || channel > 14) {
      continue;
    }

    String ssid = WiFi.SSID(i);
    String bssid = WiFi.BSSIDstr(i);
    int rssi = WiFi.RSSI(i);

    ssid.replace(",", " ");
    Serial.printf(
      "WIFI,%s,%s,%d,%d\n",
      ssid.c_str(),
      bssid.c_str(),
      rssi,
      channel
    );
  }

  WiFi.scanDelete();
}
