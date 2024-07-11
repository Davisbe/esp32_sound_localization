#include <WiFi.h>
#include <driver/i2s.h>
#include <ArduinoWebsockets.h>
#include <WiFiClientSecure.h>
#include <WebSocketsClient.h>
#include "time.h"

#include <esp_now.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()

// As there are 2 other ESP32s, we need to store only the 2
const int SlaveCnt_expected = 2;
esp_now_peer_info_t slaves[SlaveCnt_expected] = {};
esp_now_peer_info_t broadcast_slave;
int SlaveCnt = 0;
int SlaveCnt_added = 0;
bool is_this_esp_master = 1;

WebSocketsClient webSocket;
TaskHandle_t micHandle = NULL; // handle for the sending of messages in a loop
uint32_t mac_nic = ESP.getEfuseMac() >> 32; // Get the upper 32 bits which should be unique enough

const char* ssid = "Pixel_8327";
const char* password = "bigparoledrosa";

const char* websocket_server_host = "192.168.62.46";
const uint16_t websocket_server_port = 8888;  // <WEBSOCKET_SERVER_PORT>

#define USE_SERIAL Serial
#define I2S_SD 10
#define I2S_WS 11
#define I2S_SCK 12
#define I2S_PORT I2S_NUM_0

#define bufferCnt 10
#define bufferLen 1024
uint16_t sBuffer[bufferLen];

void hexdump(const void *mem, uint32_t len, uint8_t cols = 16) {
	const uint8_t* src = (const uint8_t*) mem;
	USE_SERIAL.printf("\n[HEXDUMP] Address: 0x%08X len: 0x%X (%d)", (ptrdiff_t)src, len, len);
	for(uint32_t i = 0; i < len; i++) {
		if(i % cols == 0) {
			USE_SERIAL.printf("\n[0x%08X] 0x%08X: ", (ptrdiff_t)src, i);
		}
		USE_SERIAL.printf("%02X ", *src);
		src++;
	}
	USE_SERIAL.printf("\n");
}

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    //.sample_rate = 16000,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = bufferCnt,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void connectWiFi() {
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    USE_SERIAL.println("");
    USE_SERIAL.printf(".");
  }
  USE_SERIAL.println("WiFi connected");
}

void disconnectWiFi() {
  while (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    delay(500);
  }
}

/*
__________________________________________________________________________________________

    ESP NOW Master functions
__________________________________________________________________________________________
*/

void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks(false, false, false, 300, 1);
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  USE_SERIAL.println("");
  if (scanResults == 0) {
    USE_SERIAL.println("No WiFi devices in AP Mode found");
  } else {
    USE_SERIAL.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf("Slave") == 0) {
        // SSID of interest
        USE_SERIAL.print(i + 1);
        USE_SERIAL.print(": ");
        USE_SERIAL.print(SSID);
        USE_SERIAL.print(" [");
        USE_SERIAL.print(BSSIDstr);
        USE_SERIAL.print("]");
        USE_SERIAL.print(" (");
        USE_SERIAL.print(RSSI);
        USE_SERIAL.print(")");
        USE_SERIAL.println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = 1; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    USE_SERIAL.print(SlaveCnt); USE_SERIAL.println(" Slave(s) found, processing..");
  } else {
    USE_SERIAL.println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      USE_SERIAL.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        USE_SERIAL.print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) USE_SERIAL.print(":");
      }
      USE_SERIAL.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(slaves[i].peer_addr);
      if (exists) {
        // Slave already paired.
        USE_SERIAL.println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&slaves[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          SlaveCnt_added += 1;
          USE_SERIAL.println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          USE_SERIAL.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          USE_SERIAL.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          USE_SERIAL.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          USE_SERIAL.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          USE_SERIAL.println("Peer Exists");
        } else {
          USE_SERIAL.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    USE_SERIAL.println("No Slave found to process");
  }
}

void sendTimeData() {
  USE_SERIAL.println("** Sending time data to ESP32 slaves **");
  struct timeval tv_now;
  //for (int i = 0; i < SlaveCnt_expected; i++) {
    const uint8_t peer_addr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (int ii = 0; ii < 6; ++ii ) {
      broadcast_slave.peer_addr[ii] = (uint8_t) peer_addr[ii];
    }
    esp_now_add_peer(&broadcast_slave);
    //const uint8_t *peer_addr = slaves[i].peer_addr;
    int sent_times = 0;
    unsigned long time_next = millis();
    while (sent_times < 10) {
      // if (millis() < time_next) {
      //   continue;
      // }
      // Get current time
      gettimeofday(&tv_now, NULL);
      int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
      time_next += 100;
      esp_err_t result = esp_now_send(peer_addr, (uint8_t*)&time_us, sizeof(time_us));
      USE_SERIAL.print("Send Status: ");
      if (result == ESP_OK) {
        sent_times++;
      } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        USE_SERIAL.println("ESPNOW not Init.");
      } else if (result == ESP_ERR_ESPNOW_ARG) {
        USE_SERIAL.println("Invalid Argument");
      } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
        USE_SERIAL.println("Internal Error");
      } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
        USE_SERIAL.println("ESP_ERR_ESPNOW_NO_MEM");
      } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        USE_SERIAL.println("Peer not found.");
      } else {
        USE_SERIAL.println("Not sure what happened");
      }
      delayMicroseconds(10000);
    }
    
  //}
  USE_SERIAL.println("** Done sending time **");
}

/*
__________________________________________________________________________________________

    ESP NOW Slave functions
__________________________________________________________________________________________
*/
// config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), 1, 0);
  if (!result) {
    USE_SERIAL.println("AP Config failed.");
  } else {
    USE_SERIAL.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

int64_t times_received[20]; // [time_received_at1, received_time1, time_received_at2, received_time2, .. time_received_at(n), received_time(n),]
int16_t times_received_count = 0;
void OnTimeDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len == 8) {
    int64_t rec_time = 0;
    for(int i = 0; i < 8; ++i) {
        rec_time |= ((int64_t)data[i]) << (i * 8);
    }

    if (times_received_count < 10) {
      times_received[times_received_count*2] = micros();
      times_received[times_received_count*2 + 1] = rec_time;
      times_received_count++;
    }

  }
}

void SetReceivedTime() {
  struct timeval tv_now;
  int64_t best_time_diff = 0;
  int16_t best_time_idx = 0;
  for (int i = 2; i < 20; i+=2) {
    int64_t time_diff = times_received[i] - times_received[i-2];
    USE_SERIAL.println(time_diff);
    if (time_diff < best_time_diff || best_time_diff == 0) {
      best_time_diff = time_diff;
      best_time_idx = i;
    }
  }
  int64_t new_time = times_received[best_time_idx + 1];

  gettimeofday(&tv_now, NULL);
  // cast to signed int64 still gives us 35mins of time in microseconds
  int64_t time_now = (int64_t)micros();

  new_time += (time_now - times_received[best_time_idx]);
  tv_now.tv_sec = new_time / 1000000L;
  tv_now.tv_usec = new_time % 1000000L;
  settimeofday(&tv_now, NULL);
  return;
}

/*
__________________________________________________________________________________________

    ESP NOW Slave and Master functions
__________________________________________________________________________________________
*/

void InitESPNow() {
  WiFi.disconnect();
  USE_SERIAL.println("ESPNow connecting");
  if (esp_now_init() == ESP_OK) {
    USE_SERIAL.println("ESPNow Init Success");
  }
  else {
    USE_SERIAL.print(".");
    delay(500);
    InitESPNow();
  }
}

void connectESPNOW() {
  if (is_this_esp_master == 1) {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    InitESPNow();

    while (1) {
      // scan for slaves
      ScanForSlave();
      // If Slave is found, it would be populate in `slave` variable
      // We will check if `slave` is defined and then we proceed further
      if (SlaveCnt > 0) { // check if slave channel is defined
        // `slave` is defined
        // Add slave as peer if it has not been added already
        manageSlave();
        // pair success or already paired
        if (SlaveCnt_added == SlaveCnt_expected) {
          // Send time data to all ESP32s
          sendTimeData();
          // Time synced, no need for ESP NOW anymore
          break;
        }
      }

      // wait for 1second to run the logic again
      delay(1000);
    }
  } else {
    // Will be receiving time info
    struct timeval tv_now;
    tv_now.tv_sec = (int64_t)1111111111111111 / 1000000L;
    tv_now.tv_usec = (int64_t)1111111111111111 % 1000000L;
    settimeofday(&tv_now, NULL);

    WiFi.mode(WIFI_AP);
    configDeviceAP();
    InitESPNow();
    esp_now_register_recv_cb(OnTimeDataRecv);

    while (1) {
      if (times_received_count >= 10) {
        // all 10 time values received
        SetReceivedTime();
        WiFi.mode(WIFI_STA);
        break;
      }
      delay(100);
    }
  }

  WiFi.disconnect();
  if (esp_now_deinit() == ESP_OK) {
    USE_SERIAL.println("ESPNow Off");
  }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {

	switch(type) {
		case WStype_DISCONNECTED:
			USE_SERIAL.printf("[WSc] Disconnected!\n");
      vTaskDelete(micHandle);
      micHandle = NULL;
			break;
		case WStype_CONNECTED:
			USE_SERIAL.printf("[WSc] Connected to url: %s\n", payload);

      // start sending audio
      if (micHandle == NULL) {
        xTaskCreatePinnedToCore(micTask, "micTask", 10000, NULL, 1, &micHandle, 1);
      }
			break;
		case WStype_TEXT:
			USE_SERIAL.printf("[WSc] get text: %s\n", payload);

			// send message to server
			// webSocket.sendTXT("message here");
			break;
		case WStype_BIN:
			USE_SERIAL.printf("[WSc] get binary length: %u\n", length);
			hexdump(payload, length);

			// send data to server
			// webSocket.sendBIN(payload, length);
			break;
		case WStype_ERROR:			
		case WStype_FRAGMENT_TEXT_START:
		case WStype_FRAGMENT_BIN_START:
		case WStype_FRAGMENT:
		case WStype_FRAGMENT_FIN:
			break;
	}

}

void micTask(void* parameter) {
  USE_SERIAL.println("Starting micTask");
  size_t bytesIn = 0;
  struct timeval tv_now;
  while (1) {
    // Read audio data from the I2S digital microphone
    esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
    if (result == ESP_OK) {
      gettimeofday(&tv_now, NULL);
      // Get current time
      int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
      // Create new buffer
      uint16_t* nic_plus_audio_buffer = new uint16_t[2 + 4 + bytesIn];
      // Store the NIC address in the buffer
      nic_plus_audio_buffer[0] = mac_nic & 0xFFFF;
      nic_plus_audio_buffer[1] = (mac_nic >> 16) & 0xFFFF;
      // Store the current time in the buffer
      for (int i = 2; i < 6; ++i) {
        nic_plus_audio_buffer[i] = time_us & 0xFFFF;
        time_us >>= 16;
      }
      // Copy the audio data in to the buffer
      memcpy(&nic_plus_audio_buffer[6], sBuffer, bytesIn);
      // Send the packet
      webSocket.sendBIN((const uint8_t*)nic_plus_audio_buffer, bytesIn+4+8);
      // Free memory of buffer
      delete[] nic_plus_audio_buffer;
    }
  }
}

void setup() {
  // USE_SERIAL.begin(921600);
	USE_SERIAL.begin(115200);

	//Serial.setDebugOutput(true);
	USE_SERIAL.setDebugOutput(true);

	USE_SERIAL.println();
	USE_SERIAL.println();
	USE_SERIAL.println();

	for(uint8_t t = 4; t > 0; t--) {
		USE_SERIAL.printf("[SETUP] BOOT WAIT %d...\n", t);
		USE_SERIAL.flush();
		delay(1000);
	}

  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // sync initial time for master, cause why not
  if (is_this_esp_master) {
    connectWiFi();
    // Use SNTP to sync the time (worse potential precision than using ESP-NOW)
    configTime(0, 0, "lv.pool.ntp.org", "pool.ntp.org", NULL);
    for(uint8_t t = 4; t > 0; t--) {
      USE_SERIAL.printf("[SYNC TIME] WAIT %d...\n", t);
      USE_SERIAL.flush();
      delay(2000);
    }
    disconnectWiFi();
  }

  // used for time sync
  connectESPNOW();

  connectWiFi();

  // server address, port and URL
	webSocket.begin(websocket_server_host, websocket_server_port, "/");

	// event handler
	webSocket.onEvent(webSocketEvent);

	// try every 5000 again if connection has failed
	webSocket.setReconnectInterval(5000);

}

void loop() {
  webSocket.loop();
}
