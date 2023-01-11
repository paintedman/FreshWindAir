/*
 * 01.2023 dmitriev.dd@hotmail.com
 */

// https://github.com/plerup/espsoftwareserial

// WiFiManager
#include <ArduinoJson.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <FS.h>
#include <MQTTPubSubClient.h>
#include <NTPClient.h>
#include <SoftwareSerial.h>
#include <WiFiManager.h>
#include <WiFiUdp.h>
#include <arduino-timer.h>

// Our source files
#include "indication.h"
#include "sensors.h"

/* -------------------------
 *        Definitions
 * ------------------------- */
#define ENABLE_LOGS       (1)
#define ENABLE_DEBUG_LOGS (1)

#if ENABLE_LOGS
#if ENABLE_DEBUG_LOGS
#define SERIAL_DBG(fmt, args...) Serial.printf("[%lu][DBG][%s] " fmt "\n", millis(), __FUNCTION__, ##args)
#else
#define SERIAL_DBG(fmt, args...)
#endif
#define SERIAL_ERR(fmt, args...) Serial.printf("[%lu][ERR][%s] " fmt "\n", millis(), __FUNCTION__, ##args)
#define SERIAL_MSG(fmt, args...) Serial.printf("[%lu][MSG][%s] " fmt "\n", millis(), __FUNCTION__, ##args)
#else
#define SERIAL_DBG(fmt, args...)
#define SERIAL_ERR(fmt, args...)
#define SERIAL_MSG(fmt, args...)
#endif

#define SW_VERSION "0.9.0"

/* PIN definitions */
#define BUTTON_S1_PIN (10)
#define BUTTON_S2_PIN (0)

/* Sketch parameters */
#define TIMER_READ_MHZ19   (10000L)
#define TIMER_READ_DHT22   (10000L)
#define TIMER_NOTIFY       (30000L)
#define TIMER_SEND_RESULTS (30000L)
#define TIMER_SEND_UPTIME  (30000L)
#define TIMER_READ_ADC     (60000L)
#define TIMER_CLEAR_ERRORS (86400000L)

#define LED_HOUR_ENABLE  (8)
#define LED_HOUR_DISABLE (22)

#define CO2_LEVEL_AVERAGE (700)
#define CO2_LEVEL_BAD     (1100)

/* -------------------------
 *      Code starts here
 * ------------------------- */
auto timer = timer_create_default();
FreshWindSensors sensors;
Indication leds;
WiFiClient wifiClient;
MQTTPubSubClient mqttClient;
WiFiUDP udp;
NTPClient timeClient(udp);

char hostname[]        = "FreshWindAir";
char mqtt_server[100]  = "";
char mqtt_port[5]      = "";
char mqtt_id[33]       = "";
char mqtt_user[50]     = "";
char mqtt_password[50] = "";

int adcvalue;

bool notifyUseBeeper  = false; // beep works if true
bool shouldSaveConfig = false; // flag for saving data
bool online           = true;

static String getFormattedUptime() {
    int seconds = millis() / 1000;
    int minutes = seconds / 60;
    int hours   = seconds / 3600;
    int days    = seconds / 86400;

    seconds = seconds - minutes * 60;
    minutes = minutes - hours * 60;
    hours   = hours - days * 24;

    String daysStr   = String(days);
    String hoursStr  = hours < 10 ? "0" + String(hours) : String(hours);
    String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);
    String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

    return daysStr + " days " + hoursStr + ":" + minuteStr + ":" + secondStr;
}

static String getFormattedTime() {
    int seconds = timeClient.getSeconds();
    int minutes = timeClient.getMinutes();
    int hours   = timeClient.getHours();

    String hoursStr  = hours < 10 ? "0" + String(hours) : String(hours);
    String minuteStr = minutes < 10 ? "0" + String(minutes) : String(minutes);
    String secondStr = seconds < 10 ? "0" + String(seconds) : String(seconds);

    return hoursStr + ":" + minuteStr + ":" + secondStr;
}

static bool connectMqtt() {
    wifiClient.connect(mqtt_server, mqtt_port);
    mqttClient.begin(wifiClient);
    return mqttClient.connect(mqtt_id, mqtt_user, mqtt_password);
}

static void restartMcu() {
    SERIAL_MSG("Restart in 3..2..1..");

    leds.setIndication(true, true, true);

    ESP.restart();
}

static void resetWifiSettings() {
    SERIAL_MSG("Reset WiFi settings in 3..2..1..");

    // wifiManager.resetSettings();
    restartMcu();
}

static void formatFlash() {
    SERIAL_MSG("Format flash in 3..2..1..");

    SPIFFS.format();
    restartMcu();
}

void tones(uint8_t pin, unsigned int frequency, unsigned long duration) {
    if (notifyUseBeeper) {
        pinMode(pin, OUTPUT);
        analogWriteFreq(frequency);
        analogWrite(pin, 500);
        delay(duration);
        analogWrite(pin, 0);
    }
}

static void readDeviceConfig() {
    // Check flash size
    if (ESP.getFlashChipRealSize() == ESP.getFlashChipSize()) {
        SERIAL_DBG("flash correctly configured, SPIFFS starts, IDE size: %d, match real size: %d",
                   ESP.getFlashChipSize(), ESP.getFlashChipRealSize());
    } else {
        SERIAL_DBG("flash incorrectly configured, SPIFFS cannot start, IDE size: %d, real size: %d",
                   ESP.getFlashChipSize(), ESP.getFlashChipRealSize());
    }

    if (SPIFFS.begin()) {
        SERIAL_DBG("mounted file system");
        if (SPIFFS.exists("/config.json")) {
            // file exists, reading and loading
            SERIAL_DBG("reading config file");
            File configFile = SPIFFS.open("/config.json", "r");
            if (configFile) {
                SERIAL_DBG("opened config file");
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);
                configFile.readBytes(buf.get(), size);

                DynamicJsonDocument jsonDocument(1024);
                JsonObject& json = jsonDocument.parseObject(buf.get());
                json.printTo(Serial);
                if (json.success()) {
                    SERIAL_DBG("parsed json");
                    strcpy(mqtt_server, json["mqtt_server"]);
                    strcpy(mqtt_port, json["mqtt_port"]);
                    strcpy(mqtt_id, json["mqtt_id"]);
                    strcpy(mqtt_user, json["mqtt_user"]);
                    strcpy(mqtt_password, json["mqtt_password"]);
                } else {
                    SERIAL_ERR("Failed to load json config");
                }
            }
        }
    } else {
        SERIAL_ERR("Failed to mount FS");
    }
}

void SayHello() {
    SERIAL_MSG("====== SYSTEM-STATUS ================================");
    SERIAL_MSG("Device name: %s", Hostname);
    SERIAL_MSG("Software version: %s", SW_VERSION);
    SERIAL_MSG("FreeHeap: %d", ESP.getFreeHeap());
    SERIAL_MSG("ChipId: %d", ESP.getChipId());           // ESP8266 chip IDE, int 32bit
    SERIAL_MSG("FlashChipId: %d", ESP.getFlashChipId()); // flash chip ID, int 32bit
    SERIAL_MSG("FlashChipSize: %d", ESP.getFlashChipSize());
    SERIAL_MSG("FlashChipSpeed: %d", ESP.getFlashChipSpeed());
    SERIAL_MSG("CycleCount: %d", ESP.getCycleCount()); // unsigned 32-bit
    SERIAL_MSG("Time: %s", timeClient.getFormattedTime().c_str());
    SERIAL_MSG("UpTime: %s", getFormattedUptime().c_str());

    SERIAL_MSG("====== MQTT-STATUS =================================");
    SERIAL_MSG("MQTT server: %s", mqtt_server);
    SERIAL_MSG("MQTT connected: %d", mqttClient.isConnected());
    SERIAL_MSG("Beep: %d", notifyUseBeeper);

    SERIAL_MSG("====== NETWORK-STATUS ===============================");
    SERIAL_MSG("WiFi network: %s", WiFi.SSID().c_str());
    SERIAL_MSG("WiFi status: %d", WiFi.status());
    SERIAL_MSG("RSSI: %d", WiFi.RSSI());
    SERIAL_MSG("MAC: %s", WiFi.macAddress().c_str());
    SERIAL_MSG("IP: %s", WiFi.localIP().toString().c_str());
    SERIAL_MSG("Online: %d", online);
    SERIAL_MSG("====== END-of-STATUS ================================\n");
}

/* -------------------------
 *      WiFi Manager
 * ------------------------- */

void configModeCallback(WiFiManager* myWiFiManager) {
    (void)myWiFiManager;

    // gets called when WiFiManager enters configuration mode
    SERIAL_DBG("Entered config mode");
    SERIAL_DBG("%s", WiFi.softAPIP().toString().c_str());
    // if you used auto generated SSID, print it
    SERIAL_DBG("%s", myWiFiManager->getConfigPortalSSID().c_str());

    // entered config mode, make led toggle faster
    leds.setTickerFor(LedColor::Red, 0.5);
}

void saveConfigCallback() {
    // callback notifying us of the need to save config
    SERIAL_DBG("Should save config");
    shouldSaveConfig = true;

    leds.setTickerFor(LedColor::Red, 0.2);
}

void setupWifi() {
    // WiFiManager
    // Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    // WiFi credentials will be reseted if button S1 will be pressed during boot
    int buttonS1State = digitalRead(BUTTON_S1_PIN);
    if (buttonS1State == 0) {
        SERIAL_DBG("Reset Wi-Fi settings");
        wifiManager.resetSettings();
    }

    // set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback(configModeCallback);

    // set config save notify callback
    wifiManager.setSaveConfigCallback(saveConfigCallback);

    // add all your parameters here
    WiFiManagerParameter mqtt_server_token("config", "mqtt_server", mqtt_server, 100);
    WiFiManagerParameter mqtt_port_token("config", "mqtt_port", mqtt_port, 5);
    WiFiManagerParameter mqtt_id_token("config", "mqtt_id", mqtt_id, 33);
    WiFiManagerParameter mqtt_user_token("config", "mqtt_user", mqtt_user, 50);
    WiFiManagerParameter mqtt_password_token("config", "mqtt_password", mqtt_password, 50);

    wifiManager.addParameter(&mqtt_server_token);
    wifiManager.addParameter(&mqtt_port_token);
    wifiManager.addParameter(&mqtt_id_token);
    wifiManager.addParameter(&mqtt_user_token);
    wifiManager.addParameter(&mqtt_password_token);

    // sets timeout until configuration portal gets turned off
    // useful to make it all retry or go to sleep, in seconds
    wifiManager.setTimeout(300); // 5 minutes to enter data and then ESP resets to try again.

    // fetches ssid and pass and tries to connect
    // if it does not connect it starts an access point with the specified name
    if (!wifiManager.autoConnect("FreshWindAir - tap to config")) {
        if (mqtt_server[0] != '\0') {
            SERIAL_ERR("Failed to go online for MQTT, restarting..");
            delay(2000);
            restartMcu();
        } else {
            SERIAL_ERR("Failed to go online, offline mode activated");
            online = false;
            tones(13, 2000, 50);
        }
    }

    leds.clear();

    if (online) {
        tones(13, 1500, 30);

        // 300 seconds timeout to synchronize time
        SERIAL_DBG("Start time sync client\n\r");
        timeClient.begin();

        while (timeClient.getEpochTime() < 300) {
            SERIAL_ERR("Waiting to get NTP time...");
            timeClient.forceUpdate();
            delay(1000);
        }
        timeClient.setTimeOffset(10800); // +3 hours

        // read updated parameters
        strcpy(mqtt_server, mqtt_server_token.getValue());
        strcpy(mqtt_port, mqtt_port_token.getValue());
        strcpy(mqtt_id, mqtt_id_token.getValue());
        strcpy(mqtt_user, mqtt_user_token.getValue());
        strcpy(mqtt_password, mqtt_password_token.getValue());

        if (shouldSaveConfig) {
            // save the custom parameters to FS

            // https://arduinojson.org/v6/doc/upgrade/

            SERIAL_DBG("saving config");
            DynamicJsonDocument jsonDocument(1024);
            JsonObject& json = jsonDocument.createObject();

            json["mqtt_server"]   = mqtt_server;
            json["mqtt_port"]     = mqtt_port;
            json["mqtt_id"]       = mqtt_id;
            json["mqtt_user"]     = mqtt_user;
            json["mqtt_password"] = mqtt_password;

            File configFile = SPIFFS.open("/config.json", "w");
            if (!configFile) {
                SERIAL_ERR("Failed to open config file for writing");
            }

            json.printTo(Serial);
            json.printTo(configFile);
            configFile.close();
            // end save

            delay(1000);
            SERIAL_DBG("Restart ESP to apply new WiFi settings..");
            restartMcu();
        }

        SERIAL_DBG("WiFi network: %s", WiFi.SSID().c_str());
        SERIAL_DBG("WiFi status: %d", WiFi.status());
        SERIAL_DBG("Local ip: %s", WiFi.localIP().toString().c_str());

        if (mqtt_server[0] != '\0') {
            connectMqtt();
        } else {
            SERIAL_DBG("MQTT config not set");
        }
        SERIAL_DBG("FreshWindAir is ready!");
    }
}

/* -------------------------
 *      Main functions
 * ------------------------- */

// void notify() {
//     // flag that notification has sent to user
//     static bool notificationActive = false;

//     SERIAL_DBG( "Start" );

//     int ppm = sensors.getAveragePpm();
//     if ( ppm <= CO2_LEVEL_AVERAGE ) {
//         if ( notificationActive ) {
//             SERIAL_DBG( "CO2 returns back to secure level" );
//         }
//         notificationActive = false;
//     }
//     else if ( !notificationActive && ppm >= CO2_LEVEL_BAD ) {
//         Blynk.notify( String( "CO2 level > " ) + CO2_LEVEL_BAD + ". Please Open Window." );

//         SERIAL_DBG( "Sending notify to phone. CO2 level > %d", CO2_LEVEL_BAD );

//         if ( notifyUseBeeper ) {
//             tones( 13, 1000, 50 );
//             delay( 50 );
//             tones( 13, 1000, 50 );
//             delay( 50 );
//             tones( 13, 1000, 50 );
//         }
//         notificationActive = true;
//     }

//     SERIAL_DBG( "End" );
// }

void sendUptime() {
    SERIAL_DBG("Start");

    if (mqttClient.isConnected()) {
        mqttClient.publish("device/uptime", millis() / 1000);
        mqttClient.publish("device/formatted_uptime", getFormattedUptime());
        mqttClient.publish("device/mhz19_errors", sensors.getMhz19Errors());
        mqttClient.publish("device/dht22_errors", sensors.getDht22Errors());
    }

    SERIAL_DBG("End");
}

void sendResults() {
    static int prevSentAveragePpm = 0;

    SERIAL_DBG("Start");

    if (mqttClient.isConnected()) {
        mqttClient.publish("device/humidity", sensors.getHumidity());
        mqttClient.publish("device/temperature", sensors.getTemperature());
        mqttClient.publish("device/heat_index", sensors.getHeatIndex());
        mqttClient.publish("device/average_ppm", sensors.getAveragePpm());
        mqttClient.publish("device/average_ppm_diff", sensors.getAveragePpm() - prevSentAveragePpm);
    }

    /* We should compare with previous sent value */
    prevSentAveragePpm = sensors.getAveragePpm();

#if 0

    SERIAL_DBG( "===================================================" );
    SERIAL_DBG( "UpTime: %s", getFormattedUptime().c_str() );
    SERIAL_DBG( "Time: %s", timeClient.getFormattedTime().c_str() );
    SERIAL_DBG( "Humidity: %.1f %%", h );
    SERIAL_DBG( "Temperature: %.1f C (%.1f F)", t, f );
    SERIAL_DBG( "Feels like: %.1f C", hi );
    SERIAL_DBG( "C02: %d ppm", ppm );
    SERIAL_DBG( "C02 average: %d ppm", average_ppm_sum );
    SERIAL_DBG( "===================================================" );
#endif

    SERIAL_DBG("End");
}

void updateCo2() {
    SERIAL_DBG("Start");

    sensors.updateMhz19();

    // Update led indication
    leds.clear();

    int ppm = sensors.getAveragePpm();
    if (ppm < CO2_LEVEL_AVERAGE) {
        leds.setIndication(true, false, false);
    } else if (ppm < CO2_LEVEL_BAD) {
        leds.setIndication(false, true, false);
    } else {
        leds.setIndication(false, false, true);
    }

    SERIAL_DBG("End");
}

// -----------------------------------------------
//                  SETUP AND LOOP
// -----------------------------------------------

// Setup
void setup() {
    Serial.begin(115200);

    sensors.initMhz19();
    sensors.initDht22();

    pinMode(BUTTON_S1_PIN, INPUT);
    pinMode(BUTTON_S2_PIN, INPUT);

    tones(13, 1000, 100);

    // read configuration from FS json
    readDeviceConfig();

    // Start to connect to Wifi
    setupWifi();

    timer.every(TIMER_CLEAR_ERRORS, [](void*) -> bool {
        SERIAL_DBG("Start");
        sensors.clearErrors();
        SERIAL_DBG("End"); });
    timer.every(TIMER_READ_DHT22, [](void*) -> bool {
        SERIAL_DBG("Start");
        sensors.updateDht22();
        SERIAL_DBG("End"); });
    timer.every(TIMER_READ_MHZ19, [](void*) -> bool { updateCo2(); });
    // timer.every(TIMER_NOTIFY, [](void*) -> bool { notify(); });
    timer.every(TIMER_SEND_UPTIME, [](void*) -> bool { sendUptime(); });
    timer.every(TIMER_SEND_RESULTS, [](void*) -> bool { sendResults(); });

    // Serial.setDebugOutput( true );

    ESP.wdtDisable();
}

// LOOP
void loop() {
    static bool wifilost_flag = false;
    static int wifilost_timer_start;
    static int wifilost_timer_max = 300; // 300 sec timeout for reset if WiFi connection lost

    if (WiFi.status() == WL_CONNECTED) {
        timeClient.update();

        wifilost_flag = false;

        if (mqtt_server[0] != '\0') {
            if (mqttClient.isConnected() && wifiClient.connected()) {
                mqttClient.update();
            } else {
                SERIAL_MSG("Reconnecting to MQTT server.. ");

                // FIXME: add exit after n-unsuccesfull tries.
                connectMqtt();
                SERIAL_MSG("MQTT connected: %d", mqttClient.isConnected());
            }
        }
    }

    // Check whether we lost connection to Wifi
    if (WiFi.status() != WL_CONNECTED && online) {
        int uptime = millis() / 1000;

        if (!wifilost_flag) {
            wifilost_timer_start = uptime;
            wifilost_flag        = true;
        }
        if (((uptime - wifilost_timer_start) > wifilost_timer_max) && wifilost_flag) {
            SERIAL_ERR("WiFi connection lost, restarting..");
            leds.setTickerFor(LedColor::Yellow, 1.0);

            // wifilost_flag = false;
            // restartMcu();
        }
    }

    timer.tick();
    ESP.wdtFeed();

    /* Disable led indication for evening */
    if (timeClient.getHours() >= LED_HOUR_DISABLE || timeClient.getHours() < LED_HOUR_ENABLE) {
        leds.setBrightness(LED_OFF);
    } else {
        leds.setBrightness(LED_BRIGHT);
    }

    /* Buttons handling */
    int buttonS1State = digitalRead(BUTTON_S1_PIN);
    int buttonS2State = digitalRead(BUTTON_S2_PIN);

    static unsigned long lastCalibrationTime = 0;
    static bool calibrationStatus            = false;
    if (buttonS1State == 0) {
        if (millis() - lastCalibrationTime > 5000) {
            if (calibrationStatus) {
                SERIAL_DBG("Set MHZ-19 ABC on...");

                sensors.setMhz19Abc(true);
            } else {
                SERIAL_DBG("Set MHZ-19 ABC off...");

                sensors.setMhz19Abc(false);
            }
            calibrationStatus   = !calibrationStatus;
            lastCalibrationTime = millis();
        }
    }

    if (buttonS2State == 0) {
        restartMcu();
    }

    while (Serial.available() > 0) {
        if (Serial.read() == '\r' || Serial.read() == '\n') {
            SayHello();
            tones(13, 1000, 100);
        }
    }
}
