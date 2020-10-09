/*
 * 07.2019 dmitriev.dd@hotmail.com
 * 
 * This sketch based on OpenWind project available at
 * https://github.com/zaharenkov/openwindair
 */

/*
 * Blynk values:
 * 
 * V1 - Humidity
 * V2 - Temperature in C
 * V3 - Temperature in F
 * V4 - Average CO2 ppm
 * V5 - ADC value
 * V6 - Difference of current average CO2 and previous
 * V7 - Heat index
 * V8 - Uptime as string 
 * V97 - MHZ19 errors count
 * V98 - DHT22 errors count
 * V99 - Uptime as integer in seconds
 */ 

// Comment this out to disable prints and save space
// #define BLYNK_PRINT Serial   

#include <FS.h>

//blynk
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

//RTC Time
#include <NTPClient.h>
#include <WiFiUdp.h>

// https://github.com/plerup/espsoftwareserial
#include <SoftwareSerial.h>

//WiFiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>

// Our source files
#include "indication.h"
#include "sensors.h"

/* -------------------------
 *        Definitions
 * ------------------------- */ 
#define ENABLE_LOGS         (0)
#define ENABLE_DEBUG_LOGS   (0)
#define ENABLE_BLYNK_LOGS   (1)

#if ENABLE_LOGS
    #if ENABLE_DEBUG_LOGS
        #define SERIAL_DBG( fmt, args... ) Serial.printf( "[%lu][DBG][%s] " fmt "\n", millis(), __FUNCTION__, ##args )
    #else
        #define SERIAL_DBG( fmt, args... )
    #endif
    #define SERIAL_ERR( fmt, args... ) Serial.printf( "[%lu][ERR][%s] " fmt "\n", millis(), __FUNCTION__, ##args )
    #define SERIAL_MSG( fmt, args... ) Serial.printf( "[%lu][MSG][%s] " fmt "\n", millis(), __FUNCTION__, ##args )
#else
    #define SERIAL_DBG( fmt, args... )
    #define SERIAL_ERR( fmt, args... )
    #define SERIAL_MSG( fmt, args... )
#endif

#if ENABLE_BLYNK_LOGS
    char blynk_msg[ 255 ];

    #define BLYNK_MSG( fmt, args... ) \
        do { \
            sprintf( blynk_msg, "[%s (%s)]\n" fmt "\n\n", \
                getFormattedTime().c_str(), getFormattedUptime().c_str(), ##args ); \
            terminal.print( blynk_msg ); \
            terminal.flush(); \
        } while( 0 )
#else
    #define BLYNK_MSG( fmt, ARGS )
#endif

#define SW_VERSION          "0.5.0"

/* PIN definitions */
#define BUTTON_S1_PIN       (10)
#define BUTTON_S2_PIN       (0)

/* Sketch parameters */
#define TIMER_READ_MHZ19    (10000L)
#define TIMER_READ_DHT22    (10000L)
#define TIMER_NOTIFY        (30000L)
#define TIMER_SEND_RESULTS  (30000L)
#define TIMER_SEND_UPTIME   (30000L)
#define TIMER_READ_ADC      (60000L)
#define TIMER_CLEAR_ERRORS  (86400000L)

#define LED_HOUR_ENABLE     (8)
#define LED_HOUR_DISABLE    (22)

#define CO2_LEVEL_AVERAGE   (700)
#define CO2_LEVEL_BAD       (1100)

/* -------------------------
 *      Code starts here
 * ------------------------- */ 
BlynkTimer timer;
WiFiUDP udp;
NTPClient timeClient( udp );
Indication leds;
FreshWindSensors sensors;

char Hostname[ 32 ] = "FreshWindAir";
char blynk_token[ 34 ];
char blynk_server[ 40 ];
char blynk_port[ 6 ];

int adcvalue;

bool notifyUseBeeper = false;  // beep works if true
bool shouldSaveConfig = false;  // flag for saving data
bool online = true;

static String getFormattedUptime() {
    int seconds = millis() / 1000;
    int minutes = seconds / 60;
    int hours = seconds / 3600;
    int days = seconds / 86400;

    seconds = seconds - minutes * 60;
    minutes = minutes - hours * 60;
    hours = hours - days * 24;

    String daysStr = String( days );
    String hoursStr = hours < 10 ? "0" + String( hours ) : String( hours );
    String minuteStr = minutes < 10 ? "0" + String( minutes ) : String( minutes );
    String secondStr = seconds < 10 ? "0" + String( seconds ) : String( seconds );

    return daysStr + " days " + hoursStr + ":" + minuteStr + ":" + secondStr;
}

static String getFormattedTime() {
    int seconds = timeClient.getSeconds();
    int minutes = timeClient.getMinutes();
    int hours = timeClient.getHours();

    String hoursStr = hours < 10 ? "0" + String( hours ) : String( hours );
    String minuteStr = minutes < 10 ? "0" + String( minutes ) : String( minutes );
    String secondStr = seconds < 10 ? "0" + String( seconds ) : String( seconds );

    return hoursStr + ":" + minuteStr + ":" + secondStr;
}

static bool connectBlynk() {
    _blynkWifiClient.stop();
    return _blynkWifiClient.connect( BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT );
}

BLYNK_CONNECTED() {
    Blynk.syncVirtual( V101 );
    Blynk.syncVirtual( V102 );
    Blynk.syncVirtual( V103 );
    Blynk.syncVirtual( V104 );
    Blynk.syncVirtual( V105 );
    Blynk.syncVirtual( V106 );
    Blynk.syncVirtual( V107 );
    Blynk.syncVirtual( V108 );
    Blynk.syncVirtual( V109 );
    Blynk.syncVirtual( V110 );
}

WidgetLED led1( V10 );
WidgetLED led2( V11 );
WidgetTerminal terminal( V100 );

static void restartMcu() {
    BLYNK_MSG( "Restart in 3..2..1.." );
    SERIAL_MSG( "Restart in 3..2..1.." );

    leds.setIndication( true, true, true );

    ESP.restart();
}

static void resetWifiSettings() {
    BLYNK_MSG( "Reset WiFi settings in 3..2..1.." );
    SERIAL_MSG( "Reset WiFi settings in 3..2..1.." );

    // wifiManager.resetSettings(); 
    restartMcu();
}

static void formatFlash() {
    BLYNK_MSG( "Format flash in 3..2..1.." );
    SERIAL_MSG( "Format flash in 3..2..1.." );

    SPIFFS.format();
    restartMcu();
}

void tones( uint8_t _pin, unsigned int frequency, unsigned long duration ) {
    if ( notifyUseBeeper ) {
        pinMode( _pin, OUTPUT );
        analogWriteFreq( frequency );
        analogWrite( _pin, 500 );
        delay( duration );
        analogWrite( _pin, 0 );
    }
}

static void readBlynkToken() {
    // Check flash size   
    if ( ESP.getFlashChipRealSize() == ESP.getFlashChipSize() ) {
        SERIAL_DBG( "flash correctly configured, SPIFFS starts, IDE size: %d, match real size: %d",
            ESP.getFlashChipSize(), ESP.getFlashChipRealSize() );
    }
    else {
        SERIAL_DBG( "flash incorrectly configured, SPIFFS cannot start, IDE size: %d, real size: %d",
            ESP.getFlashChipSize(), ESP.getFlashChipRealSize() );
    }

    if ( SPIFFS.begin() ) {
        SERIAL_DBG( "mounted file system" );
        if ( SPIFFS.exists( "/config.json" ) ) {
            //file exists, reading and loading
            SERIAL_DBG( "reading config file" );
            File configFile = SPIFFS.open( "/config.json", "r" );
            if ( configFile ) {
                SERIAL_DBG( "opened config file" );
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf( new char[ size ] );
                configFile.readBytes( buf.get(), size );

                DynamicJsonBuffer jsonBuffer;
                JsonObject& json = jsonBuffer.parseObject( buf.get() );
                json.printTo( Serial );
                if ( json.success() ) {
                    SERIAL_DBG( "parsed json" );
                    strcpy( blynk_token, json[ "blynk_token" ] );
                } else {
                    SERIAL_ERR( "Failed to load json config" );
                }
            }
        }
    }
    else {
        SERIAL_ERR( "Failed to mount FS" );
    }
}

void SayHello() {
    SERIAL_MSG( "====== SYSTEM-STATUS ================================" );
    SERIAL_MSG( "Device name: %s", Hostname );
    SERIAL_MSG( "Software version: %s", SW_VERSION );
    SERIAL_MSG( "FreeHeap: %d", ESP.getFreeHeap() );
    SERIAL_MSG( "ChipId: %d", ESP.getChipId() );               //ESP8266 chip IDE, int 32bit
    SERIAL_MSG( "FlashChipId: %d", ESP.getFlashChipId() );     //flash chip ID, int 32bit
    SERIAL_MSG( "FlashChipSize: %d", ESP.getFlashChipSize() );
    SERIAL_MSG( "FlashChipSpeed: %d", ESP.getFlashChipSpeed() );
    SERIAL_MSG( "CycleCount: %d", ESP.getCycleCount() );       //unsigned 32-bit
    SERIAL_MSG( "Time: %s", timeClient.getFormattedTime().c_str() );
    SERIAL_MSG( "UpTime: %s", getFormattedUptime().c_str() );

    SERIAL_MSG( "====== BLYNK-STATUS =================================" );
    SERIAL_MSG( "Blynk token: %s", blynk_token );
    SERIAL_MSG( "Blynk connected: %d", Blynk.connected() );
    SERIAL_MSG( "Beep: %d", notifyUseBeeper );

    SERIAL_MSG( "====== NETWORK-STATUS ===============================" );
    SERIAL_MSG( "WiFi network: %s", WiFi.SSID().c_str() );
    SERIAL_MSG( "WiFi status: %d", WiFi.status() );
    SERIAL_MSG( "RSSI: %d", WiFi.RSSI() );
    SERIAL_MSG( "MAC: %s", WiFi.macAddress().c_str() );
    SERIAL_MSG( "IP: %s", WiFi.localIP().toString().c_str() );
    SERIAL_MSG( "Online: %d", online );
    SERIAL_MSG( "====== END-of-STATUS ================================\n" );

}

/* -------------------------
 *      WiFi Manager 
 * ------------------------- */ 

void configModeCallback( WiFiManager* myWiFiManager ) {
    ( void ) myWiFiManager;
  
    //gets called when WiFiManager enters configuration mode
    SERIAL_DBG( "Entered config mode" );
    SERIAL_DBG( "%s", WiFi.softAPIP().toString().c_str() );
    //if you used auto generated SSID, print it
    SERIAL_DBG( "%s", myWiFiManager->getConfigPortalSSID().c_str() );

    //entered config mode, make led toggle faster
    leds.setTickerFor( LedColor::Red, 0.5 );
}

void saveConfigCallback() {  
    //callback notifying us of the need to save config
    SERIAL_DBG( "Should save config" );
    shouldSaveConfig = true;

    leds.setTickerFor( LedColor::Red, 0.2 );
}

void setupWifi() {
    WiFiManagerParameter custom_blynk_token( "blynk", "blynk token", blynk_token, 33 );   

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    // WiFi credentials will be reseted if button S1 will be pressed during boot
    int buttonS1State = digitalRead( BUTTON_S1_PIN );
    if ( buttonS1State == 0 ) {
        SERIAL_DBG( "Reset Wi-Fi settings" );
        wifiManager.resetSettings();
    }

    //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
    wifiManager.setAPCallback( configModeCallback );

    //set config save notify callback
    wifiManager.setSaveConfigCallback( saveConfigCallback );

    wifiManager.addParameter( &custom_blynk_token );   //add all your parameters here

    //sets timeout until configuration portal gets turned off
    //useful to make it all retry or go to sleep, in seconds
    wifiManager.setTimeout( 300 );   // 5 minutes to enter data and then ESP resets to try again.

    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name   
    if ( !wifiManager.autoConnect( "FreshWindAir - tap to config" ) ) {
        if ( blynk_token[ 0 ] != '\0' ) {
            SERIAL_ERR( "Failed to go online for Blynk, restarting.." );
            delay( 2000 );
            restartMcu();
        } else {
            SERIAL_ERR( "Failed to go online, offline mode activated" );
            online = false;
            tones( 13, 2000, 50 );
        }
    }

    leds.clear();

    if ( online ) {
        tones( 13, 1500, 30 );

        // 300 seconds timeout to synchronize time
        SERIAL_DBG( "Start time sync client\n\r" );
        timeClient.begin();
        
        while( timeClient.getEpochTime() < 300 ) {
            SERIAL_ERR( "Waiting to get NTP time..." );
            timeClient.forceUpdate();
            delay( 1000 );
        }
        timeClient.setTimeOffset( 10800 );  // +3 hours

        strcpy( blynk_token, custom_blynk_token.getValue() );    //read updated parameters

        if ( shouldSaveConfig ) {      
            //save the custom parameters to FS
            SERIAL_DBG( "saving config" );
            DynamicJsonBuffer jsonBuffer;
            JsonObject & json = jsonBuffer.createObject();
            json[ "blynk_token" ] = blynk_token;

            File configFile = SPIFFS.open( "/config.json", "w" );
            if ( !configFile ) {
                SERIAL_ERR( "Failed to open config file for writing" );
            }

            json.printTo( Serial );
            json.printTo( configFile );
            configFile.close();
            //end save

            delay( 1000 );
            SERIAL_DBG( "Restart ESP to apply new WiFi settings.." );
            restartMcu();
        }

        SERIAL_DBG( "WiFi network: %s", WiFi.SSID().c_str() );
        SERIAL_DBG( "WiFi status: %d", WiFi.status() );
        SERIAL_DBG( "Local ip: %s", WiFi.localIP().toString().c_str() );

        if ( blynk_token[ 0 ] != '\0' ) {
            connectBlynk();
            Blynk.config( blynk_token );
            Blynk.connect();
            
            SERIAL_DBG( "blynk token: %s", blynk_token );
        } else {
            SERIAL_DBG( "blynk auth token not set" );
        }
        SERIAL_DBG( "FreshWindAir is ready!" );
    }
}

/* -------------------------
 *      Main functions
 * ------------------------- */ 

void notify() {
    // flag that notification has sent to user
    static bool notificationActive = false;       

    SERIAL_DBG( "Start" );
  
    int ppm = sensors.getAveragePpm();
    if ( ppm <= CO2_LEVEL_AVERAGE ) {
        if ( notificationActive ) {
            BLYNK_MSG( "CO2 returns back to secure level" );
            SERIAL_DBG( "CO2 returns back to secure level" );
        }
        notificationActive = false;
    }
    else if ( !notificationActive && ppm >= CO2_LEVEL_BAD ) {
        Blynk.notify( String( "CO2 level > " ) + CO2_LEVEL_BAD + ". Please Open Window." );

        BLYNK_MSG( "Sending notify to phone. ppm > %d", CO2_LEVEL_BAD );
        SERIAL_DBG( "Sending notify to phone. CO2 level > %d", CO2_LEVEL_BAD );

        if ( notifyUseBeeper ) {
            tones( 13, 1000, 50 );
            delay( 50 );
            tones( 13, 1000, 50 );
            delay( 50 );
            tones( 13, 1000, 50 );
        }
        notificationActive = true;
    }

    SERIAL_DBG( "End" );    
}

void sendUptime() {
    SERIAL_DBG( "Start" );  

    if ( Blynk.connected() ) {
        Blynk.virtualWrite( V99, millis() / 1000 );
        Blynk.virtualWrite( V8, getFormattedUptime() );
        Blynk.virtualWrite( V97, sensors.getMhz19Errors() );
        Blynk.virtualWrite( V98, sensors.getDht22Errors() );
    }

    SERIAL_DBG( "End" );        
}

void sendResults() {
    static int prevSentAveragePpm = 0;

    SERIAL_DBG( "Start" );

    Blynk.virtualWrite( V1, sensors.getHumidity() );
    Blynk.virtualWrite( V2, sensors.getTemperature() );
    Blynk.virtualWrite( V3, sensors.getTemperatureF() );
    Blynk.virtualWrite( V7, sensors.getHeatIndex() );
    Blynk.virtualWrite( V4, sensors.getAveragePpm() );
    Blynk.virtualWrite( V6, sensors.getAveragePpm() - prevSentAveragePpm );

    /* We should compare with previous sent value */
    prevSentAveragePpm = sensors.getAveragePpm();

#if 0
    BLYNK_MSG( "Home weather : %.1f %%, %.1f C, %.1f C", h, t, hi );
    BLYNK_MSG( "C02 average  : %d ppm (diff: %d ppm)", average_ppm_sum, average_ppm_diff );

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

    SERIAL_DBG( "End" );
}

void updateCo2() {
    SERIAL_DBG( "Start" );

    sensors.updateMhz19();

    // Update led indication 
    leds.clear();

    int ppm = sensors.getAveragePpm();
    if ( ppm < CO2_LEVEL_AVERAGE ) {
        leds.setIndication( true, false, false );
    } else if ( ppm < CO2_LEVEL_BAD ) {
        leds.setIndication( false, true, false );
    } else {
        leds.setIndication( false, false, true );
    }

    SERIAL_DBG( "End" );
}

void updateTemp() {
    SERIAL_DBG( "Start" );

    sensors.updateDht22();

    SERIAL_DBG( "End" );
}

void clearErrors() {
    SERIAL_DBG( "Start" );

    sensors.clearErrors();

    SERIAL_DBG( "End" );
}

// -----------------------------------------------
//                  SETUP AND LOOP
// -----------------------------------------------

// Setup
void setup() {
    Serial.begin( 115200 );

    sensors.initMhz19();
    sensors.initDht22();
    
    pinMode( BUTTON_S1_PIN, INPUT );
    pinMode( BUTTON_S2_PIN, INPUT );

    tones( 13, 1000, 100 );

    //read configuration from FS json
    readBlynkToken();

    // Start to connect to Wifi
    setupWifi();

    timer.setInterval( TIMER_CLEAR_ERRORS, clearErrors );
    timer.setInterval( TIMER_READ_MHZ19, updateCo2 );
    timer.setInterval( TIMER_READ_DHT22, updateTemp );
    timer.setInterval( TIMER_NOTIFY, notify );
    timer.setInterval( TIMER_SEND_UPTIME, sendUptime );
    timer.setInterval( TIMER_SEND_RESULTS, sendResults );
    
    // Serial.setDebugOutput( true );
    
    BLYNK_MSG( "------ FreshWindAir, v.%s ------", SW_VERSION );

    ESP.wdtDisable();
}

// LOOP
void loop() {
    static bool wifilost_flag = false;
    static int wifilost_timer_start;
    static int wifilost_timer_max = 300; // 300 sec timeout for reset if WiFi connection lost
  
    if ( WiFi.status() == WL_CONNECTED ) {
        timeClient.update();
      
        wifilost_flag = false;

        if ( blynk_token[ 0 ] != '\0' ) {
            if ( Blynk.connected() && _blynkWifiClient.connected() ) {
                Blynk.run();
            } else {
                SERIAL_DBG( "Reconnecting to blynk.. " );
                SERIAL_DBG( "Blynk connected: %d", Blynk.connected() );
                // if ( !_blynkWifiClient.connected() ) {
                //     connectBlynk();
                //     return;
                // }

                //FIXME: add exit after n-unsuccesfull tries.
                Blynk.connect( 5000 );
                SERIAL_DBG( "Blynk connected: %d", Blynk.connected() );
            }
        }
    }

    // Check whether we lost connection to Wifi
    if ( WiFi.status() != WL_CONNECTED && online ) {
        int uptime = millis() / 1000;

        if ( !wifilost_flag ) {
            wifilost_timer_start = uptime;
            wifilost_flag = true;
        }
        if ( ( ( uptime - wifilost_timer_start ) > wifilost_timer_max ) && wifilost_flag ) {
            SERIAL_ERR( "WiFi connection lost, restarting.." );
            leds.setTickerFor( LedColor::Yellow, 1.0 );
            
            // wifilost_flag = false;
            // restartMcu();
        }
    }

    timer.run();
    ESP.wdtFeed();

    /* Disable led indication for evening */
    if ( timeClient.getHours() >= LED_HOUR_DISABLE || timeClient.getHours() < LED_HOUR_ENABLE ) {
        leds.setBrightness( LED_OFF );
    } else {
        leds.setBrightness( LED_BRIGHT );
    }

    /* Buttons handling */
    int buttonS1State = digitalRead( BUTTON_S1_PIN );
    int buttonS2State = digitalRead( BUTTON_S2_PIN );

    static unsigned long lastCalibrationTime = 0;
    static bool calibrationStatus = false;
    if ( buttonS1State == 0 ) {
        if ( millis() - lastCalibrationTime > 5000 ) {         
            if ( calibrationStatus ) {
                SERIAL_DBG( "Set MHZ-19 ABC on..." );
                BLYNK_MSG( "Set MHZ-19 ABC on..." );

                sensors.setMhz19Abc( true );
            } else {
                SERIAL_DBG( "Set MHZ-19 ABC off..." );
                BLYNK_MSG( "Set MHZ-19 ABC off..." );

                sensors.setMhz19Abc( false );
            }
            calibrationStatus = !calibrationStatus;  
            lastCalibrationTime = millis();
        }
    }

    if ( buttonS2State == 0 ) {
        restartMcu();
    }

    while ( Serial.available() > 0 ) {
        if ( Serial.read() == '\r' || Serial.read() == '\n' ) {
            SayHello();
            tones( 13, 1000, 100 );
        }
    }
}
