/**
 * @file
 *
 * EspNowUnicast.ino demonstrates how to transmit unicast ESP-NOW messages with @c WifiEspNow .
 * You need two ESP8266 or ESP32 devices to run this example.
 *
 * Unicast communication requires the sender to specify the MAC address of the recipient.
 * Thus, you must modify this program for each device.
 *
 * The recommended workflow is:
 * @li 1. Flash the program onto device A.
 * @li 2. Run the program on device A, look at serial console for its MAC address.
 * @li 3. Copy the MAC address of device A, paste it in the @c PEER variable below.
 * @li 4. Flash the program that contains A's MAC address onto device B.
 * @li 5. Run the program on device A, look at serial console for its MAC address.
 * @li 6. Copy the MAC address of device B, paste it in the @c PEER variable below.
 * @li 7. Flash the program that contains B's MAC address onto device A.
 */

#include <WifiEspNow.h>
#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#elif defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

#define CIRCULAR_BUFFER_INT_SAFE
#include <CircularBuffer.h>

CircularBuffer<uint8_t, 20000> cbuffer;

static uint8_t MAC[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};

// The recipient MAC address. It must be modified for each device.
// static uint8_t PEER[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0x00};
static uint8_t PEERS[][6] = {{0xF6, 0xCF, 0xA2, 0x60, 0x8C, 0xE8}, {0x4A, 0x3F, 0xDA, 0x86, 0xA3, 0x07}};

void printReceivedMessage(const uint8_t mac[WIFIESPNOW_ALEN], const uint8_t *buf, size_t count,
                          void *arg)
{
  auto now = millis();

  for (int i = 0; i < WIFIESPNOW_ALEN; i++)
  {
    cbuffer.push(mac[i]);
  }
  cbuffer.push(count >> 8);
  cbuffer.push(count);
  cbuffer.push(now >> 24);
  cbuffer.push(now >> 16);
  cbuffer.push(now >> 8);
  cbuffer.push(now >> 0);
  for (size_t i = 0; i < count; i++)
  {
    cbuffer.push(buf[i]);
  }

  // digitalWrite(LED_BUILTIN, LOW);
  // delay(1);
  // digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  // Serial.setRxBufferSize(0);
  // Serial.println();

  wifi_set_macaddr(SOFTAP_IF, MAC);

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.disconnect();
  WiFi.softAP("ESPNOW", nullptr, 3);
  WiFi.softAPdisconnect(false);
  WiFi.setSleep(false);
  // WiFi must be powered on to use ESP-NOW unicast.
  // It could be either AP or STA mode, and does not have to be connected.
  // For best results, ensure both devices are using the same WiFi channel.

  // Serial.print("MAC address of this node is ");
  // Serial.println(WiFi.softAPmacAddress());

  uint8_t mac[6];
  WiFi.softAPmacAddress(mac);
  // Serial.println();
  // Serial.println("You can paste the following into the program for the other device:");
  // Serial.printf("static uint8_t PEER[]{0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X};\n", mac[0],
  //               mac[1], mac[2], mac[3], mac[4], mac[5]);
  // Serial.println();

  bool ok = WifiEspNow.begin();
  if (!ok)
  {
    // Serial.println("WifiEspNow.begin() failed");
    ESP.restart();
  }

  WifiEspNow.onReceive(printReceivedMessage, nullptr);

  for (auto peer : PEERS)
  {
    ok &= WifiEspNow.addPeer(peer);
  }
  if (!ok)
  {
    // Serial.println("WifiEspNow.addPeer() failed");
    ESP.restart();
  }
}

void loop()
{
  static bool waiting_for_buffer = false;
  static uint8_t mac[WIFIESPNOW_ALEN];
  static uint16_t len = 0;

  auto available = Serial.available();

  if (waiting_for_buffer)
  {
    if (available >= len)
    {
      uint8_t buf[1024];
      Serial.readBytes(buf, len);
      WifiEspNow.send(mac, buf, len);
      waiting_for_buffer = false;
    }
  }
  else
  {
    if (available >= WIFIESPNOW_ALEN + 2)
    {
      Serial.readBytes(mac, sizeof(mac));
      len = (Serial.read() << 8) | Serial.read();
      waiting_for_buffer = true;
    }
  }

  while (!cbuffer.isEmpty())
  {
    Serial.write(cbuffer.first());
    cbuffer.shift();
  }
}