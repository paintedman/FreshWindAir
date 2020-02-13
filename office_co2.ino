/*
 * 07.2019 dmitriev.dd@hotmail.com
 * 
 * This sketch based on OpenWind project available at
 * https://github.com/zaharenkov/openwindair
 */

/*
 * Blynk values:
 * 
 * V4 - Average CO2 ppm
 * V6 - Difference of current average CO2 and previous
 * V8 - Uptime as string 
 * V97 - MHZ19 errors count
 * V98 - DHT22 errors count
 * V99 - Uptime as integer in seconds
 */ 

// Comment this out to disable prints and save space
// #define BLYNK_PRINT Serial   

#include <FS.h>
#include <string.h>
#include <vector>

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

//LED ticker
#include <Ticker.h>

/* -------------------------
 *        Definitions
 * ------------------------- */ 

#define ENABLE_LOGS         (1)
#define ENABLE_DEBUG_LOGS   (1)

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

#define SW_VERSION          "0.0.1"

#define BLYNK_GREEN         "#23C48E"
#define BLYNK_BLUE          "#04C0F8"
#define BLYNK_YELLOW        "#ED9D00"
#define BLYNK_RED           "#D3435C"
#define BLYNK_DARK_BLUE     "#5F7CD8"

/* PIN definitions */


/* Sketch parameters */
#define FILTER_POINTS       (6)

#define TIMER_MAIN_CYCLE    (10000L)
#define TIMER_READ_MHZ19    (10000L)
#define TIMER_NOTIFY        (20000L)
#define TIMER_SEND_RESULTS  (20000L)
#define TIMER_SEND_UPTIME   (10000L)
#define TIMER_CLEAR_ERRORS  (86400000L)

#define CO2_LEVEL_AVERAGE   (700)
#define CO2_LEVEL_POOR      (1100)

/* -------------------------
 *      Code starts here
 * ------------------------- */ 

Ticker ticker;
SoftwareSerial co2Serial( D6, D5, false, 256 );
BlynkTimer timer;
WiFiUDP ntpUDP;
NTPClient timeClient( ntpUDP, 10800 );  // +3 hours time offset

char Hostname[ 32 ] = "OfficeFreshAir";

char blynk_token[ 34 ];
char blynk_server[ 40 ];
char blynk_port[ 6 ];

int mhz19_errors = 0;
int ppm;
int average_ppm_sum;
int average_ppm_prev;
int average_ppm_diff;
std::vector<int> ppm_values( FILTER_POINTS, 0 );

bool notify_flag = false;       // flag that notification sent to user
bool shouldSaveConfig = false;  // flag for saving data
bool online = true;

// command to ask for data
const byte MHZ19Cmd_getCO2[9] = { 0xFF, 0x01, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A };
const byte MHZ19Cmd_setRange2k[9] = { 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F };
const byte MHZ19Cmd_setAbcOff[9] = { 0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86 };
const byte MHZ19Cmd_setAbcOn[9] = { 0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6 };

static bool connectBlynk()
{
    _blynkWifiClient.stop();
    return _blynkWifiClient.connect( BLYNK_DEFAULT_DOMAIN, BLYNK_DEFAULT_PORT );
}

BLYNK_CONNECTED()
{
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

static String getFormattedUptime() 
{
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

static String getFormattedTime()
{
    int seconds = timeClient.getSeconds();
    int minutes = timeClient.getMinutes();
    int hours = timeClient.getHours();

    String hoursStr = hours < 10 ? "0" + String( hours ) : String( hours );
    String minuteStr = minutes < 10 ? "0" + String( minutes ) : String( minutes );
    String secondStr = seconds < 10 ? "0" + String( seconds ) : String( seconds );

    return hoursStr + ":" + minuteStr + ":" + secondStr;
}

static byte getCRC( byte inBytes[], int size )
{
    /* as shown in datasheet */
    byte x = 0, CRC = 0;

    for ( x = 1; x < size - 1; x++ )
    {
        CRC += inBytes[ x ];
    }
    CRC = 255 - CRC;
    CRC++;

    return CRC;
}

static void restartMcu()
{
    BLYNK_MSG( "Restart in 3..2..1.." );
    SERIAL_MSG( "Restart in 3..2..1.." );
   
    ESP.restart();
}

static void resetWifiSettings() 
{
    BLYNK_MSG( "Reset WiFi settings in 3..2..1.." );
    SERIAL_MSG( "Reset WiFi settings in 3..2..1.." );

    // wifiManager.resetSettings(); 
    restartMcu();
}

static void formatFlash() 
{
    BLYNK_MSG( "Format flash in 3..2..1.." );
    SERIAL_MSG( "Format flash in 3..2..1.." );

    SPIFFS.format();
    restartMcu();
}

static int getAverage( std::vector<int> v ) 
{
    int sum = 0;
    int count = 0;
    for ( size_t i = 0; i < v.size(); ++i ) 
    {
        if ( v[ i ] != 0 )
        {
            sum += v[ i ];
            count++;
        }            
    }
    return sum / count;
}

BLYNK_WRITE( V101 )
{
    int v101 = param.asInt(); // assigning incoming value from pin V10x to a variable
    if ( v101 == 1 )
    {
        restartMcu();
    }
}

BLYNK_WRITE( V102 )
{
    int v102 = param.asInt();
    if ( v102 == 1 )
    {
        resetWifiSettings();
    }
}

BLYNK_WRITE( V103 )
{
    int v103 = param.asInt();
    if ( v103 == 1 )
    {
        formatFlash();
    }
}

/* -------------------------
 *      WiFi Manager cb
 * ------------------------- */ 

void configModeCallback( WiFiManager* myWiFiManager )
{
    ( void ) myWiFiManager;
  
    //gets called when WiFiManager enters configuration mode
    SERIAL_DBG( "Entered config mode" );
    SERIAL_DBG( "%s", WiFi.softAPIP().toString().c_str() );
    //if you used auto generated SSID, print it
    SERIAL_DBG( "%s", myWiFiManager->getConfigPortalSSID().c_str() );
}

void saveConfigCallback()
{  
    //callback notifying us of the need to save config
    SERIAL_DBG( "Should save config" );
    shouldSaveConfig = true;
}

/* -------------------------
 *      Main functions
 * ------------------------- */ 

void notify()
{
    SERIAL_DBG( "Start" );
  
    if ( average_ppm_sum <= CO2_LEVEL_AVERAGE )
    {
        if ( notify_flag )
        {
            BLYNK_MSG( "CO2 returns back to secure level" );
            SERIAL_DBG( "CO2 returns back to secure level" );
        }
        notify_flag = false;
    }
    else if ( !notify_flag && average_ppm_sum >= CO2_LEVEL_POOR )
    {
        Blynk.notify( String( "CO2 level > " ) + CO2_LEVEL_POOR + ". Please Open Window." );

        BLYNK_MSG( "Sending notify to phone. ppm > %d", CO2_LEVEL_POOR );
        SERIAL_DBG( "Sending notify to phone. CO2 level > %d", CO2_LEVEL_POOR );

        notify_flag = true;
    }

    SERIAL_DBG( "End" );    
}

int readCO2()
{
    byte response[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /* Clear RX buffer before sending command */
    SERIAL_DBG( "data available before command: %d", co2Serial.available() );
    while ( co2Serial.available() > 0 ) 
    {
        SERIAL_DBG( "remove 0x%02x byte from buffer", co2Serial.peek() );
        co2Serial.read();
    } 

    co2Serial.write( MHZ19Cmd_getCO2, 9 ); 
    co2Serial.flush();

    int len = co2Serial.readBytes( response, 9 );
    if ( len != 9 )
    {
        SERIAL_ERR( "Wrong resp length: %x!", len );
        BLYNK_MSG( "Wrong resp len from MHZ-19: 0x%0x", len );

        return -1;
    }
    if ( response[0] != 0xFF )
    {
        SERIAL_ERR( "Wrong starting byte from CO2 sensor: %x!", response[ 0 ] );
        BLYNK_MSG( "Wrong starting byte from CO2 sensor: %x!", response[ 0 ] );
        
        return -1;
    }
    if ( response[1] != 0x85 )
    {
        SERIAL_ERR( "Wrong command from CO2 sensor! " );
        BLYNK_MSG( "Wrong command MHZ-19" );
        
        return -1;
    }
    byte crc = getCRC( response, 9 );
    if ( response[ 8 ] != crc )
    {
        SERIAL_ERR( "Wrong CRC from CO2 sensor!" );
        BLYNK_MSG( "Wrong CRC MHZ-19" );
        
        return -1;
    }
    return ( response[ 4 ] << 8 ) | response[ 5 ];
}

void readMHZ19()
{  
    SERIAL_DBG( "Start" );   

    int i = 0;
    bool MHZreadOK = false;
    ppm = -1;
    while ( ppm == -1 && i < 2 )
    {
        ppm = readCO2();

        if ( ppm != -1 )
        {
            MHZreadOK = true;
        }
        else
        {
            delay( 100 );
            i++;
        }
    }
    
    if ( MHZreadOK )
    {
        led2.on();
        led2.setColor( BLYNK_GREEN );
        SERIAL_DBG( "Reading MHZ19 sensor ok, value: %d", ppm );

        /* Update average value if read ok */
        ppm_values.erase( ppm_values.begin() );
        ppm_values.push_back( ppm );
    
        average_ppm_sum = getAverage( ppm_values );
        average_ppm_diff = average_ppm_sum - average_ppm_prev;
    
        /* Stop led blinking if active */
        if ( ticker.active() )
        {
            ticker.detach();
        }
        /* Set led indication */
        if ( average_ppm_sum < CO2_LEVEL_AVERAGE )
        {  
            led1.on();
            led1.setColor( BLYNK_GREEN );
        }
        else if ( average_ppm_sum < CO2_LEVEL_POOR )
        {    
            led1.on();
            led1.setColor( BLYNK_YELLOW );
        }
        else 
        {   
            led1.on();
            led1.setColor( BLYNK_RED );
        }
    }
    else
    {
        mhz19_errors++;
        
        led2.on();
        led2.setColor( BLYNK_YELLOW );

        SERIAL_ERR( "MHZ19 failed at %s", getFormattedUptime().c_str() );        
        BLYNK_MSG( "MHZ19 failed" );
    }

    SERIAL_DBG( "End" );
}

void SayHello()
{
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

    SERIAL_MSG( "====== NETWORK-STATUS ===============================" );
    SERIAL_MSG( "WiFi network: %s", WiFi.SSID().c_str() );
    SERIAL_MSG( "WiFi status: %d", WiFi.status() );
    SERIAL_MSG( "RSSI: %d", WiFi.RSSI() );
    SERIAL_MSG( "MAC: %s", WiFi.macAddress().c_str() );
    SERIAL_MSG( "IP: %s", WiFi.localIP().toString().c_str() );
    SERIAL_MSG( "Online: %d", online );

    SERIAL_MSG( "====== END-of-STATUS ================================\n" );

}

void sendUptime()
{
    SERIAL_DBG( "Start" );    
  
    if ( Blynk.connected() )
    {
        Blynk.virtualWrite( V99, millis() / 1000 );
        Blynk.virtualWrite( V8, getFormattedUptime() );

        Blynk.virtualWrite( V97, mhz19_errors );
    }

    SERIAL_DBG( "End" );        
}

void sendResults()
{
    SERIAL_DBG( "Start" );    
  
    /* MHZ-19 info */
    Blynk.virtualWrite( V4, average_ppm_sum );
    Blynk.virtualWrite( V6, average_ppm_diff );

    /* We should compare with previous sent value */
    average_ppm_prev = average_ppm_sum;

#if 0
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

void clearErrors()
{
    mhz19_errors = 0;
}

void mainCycle() 
{
    readMHZ19();
    notify();
    sendUptime();
    sendResults();
}

// Setup
void setup()
{
    Serial.begin( 115200 );
    
    co2Serial.begin( 9600 );
    co2Serial.write( MHZ19Cmd_setRange2k, 9 );
    co2Serial.write( MHZ19Cmd_setAbcOn, 9 );

    /* Wait MHZ-19 to become ready */
    delay( 5000 );

    // Check flash size   
    if ( ESP.getFlashChipRealSize() == ESP.getFlashChipSize() )
    {
        SERIAL_DBG( "flash correctly configured, SPIFFS starts, IDE size: %d, match real size: %d",
            ESP.getFlashChipSize(), ESP.getFlashChipRealSize() );
    }
    else
    {
        SERIAL_DBG( "flash incorrectly configured, SPIFFS cannot start, IDE size: %d, real size: %d",
            ESP.getFlashChipSize(), ESP.getFlashChipRealSize() );
    }

    //read configuration from FS json
    SERIAL_DBG( "mounting FS..." );

    if ( SPIFFS.begin() )
    {
        SERIAL_DBG( "mounted file system" );
        if ( SPIFFS.exists( "/config.json" ) )
        {
            //file exists, reading and loading
            SERIAL_DBG( "reading config file" );
            File configFile = SPIFFS.open( "/config.json", "r" );
            if ( configFile )
            {
                SERIAL_DBG( "opened config file" );
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes( buf.get(), size );
                DynamicJsonBuffer jsonBuffer;
                JsonObject & json = jsonBuffer.parseObject( buf.get() );
                json.printTo( Serial );
                if ( json.success() )
                {
                    SERIAL_DBG( "parsed json" );

                    strcpy( blynk_token, json[ "blynk_token" ] );
                }
                else
                {
                    SERIAL_ERR( "Failed to load json config" );
                }
            }
        }
    }
    else
    {
        SERIAL_ERR( "Failed to mount FS" );
    }
    //end read

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length

    WiFiManagerParameter custom_blynk_token( "blynk", "blynk token", blynk_token, 33 );   // was 32 length ???

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

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

    if ( !wifiManager.autoConnect( "OfficeFreshAir - tap to config" ) )
    {
        if ( blynk_token[ 0 ] != '\0' )
        {
            SERIAL_ERR( "Failed to go online for Blynk, restarting.." );
            delay( 2000 );
            ESP.restart();
        }
        else
        {
            SERIAL_ERR( "Failed to go online, offline mode activated" );
            online = false;
        }
    }

    ticker.detach();

    if ( online )
    {
        strcpy( blynk_token, custom_blynk_token.getValue() );    //read updated parameters

        if ( shouldSaveConfig )
        {      
            //save the custom parameters to FS
            SERIAL_DBG( "saving config" );
            DynamicJsonBuffer jsonBuffer;
            JsonObject & json = jsonBuffer.createObject();
            json[ "blynk_token" ] = blynk_token;

            File configFile = SPIFFS.open( "/config.json", "w" );
            if ( !configFile )
            {
                SERIAL_ERR( "Failed to open config file for writing" );
            }

            json.printTo( Serial );
            json.printTo( configFile );
            configFile.close();
            //end save

            delay( 1000 );
            SERIAL_DBG( "Restart ESP to apply new WiFi settings.." );
            ESP.restart();
        }

        SERIAL_DBG( "WiFi network: %s", WiFi.SSID().c_str() );
        SERIAL_DBG( "WiFi status: %d", WiFi.status() );
        SERIAL_DBG( "Local ip: %s", WiFi.localIP().toString().c_str() );

        if ( blynk_token[ 0 ] != '\0' )
        {
            connectBlynk();
            Blynk.config( blynk_token );
            Blynk.connect();
            
            SERIAL_DBG( "blynk token: %s", blynk_token );
        }
        else
        {
            SERIAL_DBG( "blynk auth token not set" );

            SERIAL_DBG( "Reset Wi-Fi settings" );
            wifiManager.resetSettings();

            restartMcu();            
        }

        SERIAL_DBG( "OfficeFreshAir is ready!" );
        
        SERIAL_DBG( "Start time sync client\n\r" );
        timeClient.begin();
    }

    timer.setInterval( TIMER_MAIN_CYCLE, mainCycle );
    timer.setInterval( TIMER_CLEAR_ERRORS, clearErrors );

    BLYNK_MSG( "------ OfficeFreshAir started! ------" );
    BLYNK_MSG( "------     version: %s    ------", SW_VERSION );

    ESP.wdtDisable();
}

// LOOP
void loop()
{
    static bool wifilost_flag = false;
    static int wifilost_timer_start;
    static int wifilost_timer_max = 60; // 60 sec timeout for reset if WiFi connection lost
  
    if ( WiFi.status() == WL_CONNECTED )
    {
        timeClient.update();
      
        wifilost_flag = false;

        if ( blynk_token[ 0 ] != '\0' )
        {
            if ( Blynk.connected() && _blynkWifiClient.connected() )
            {
                Blynk.run();
            }
            else
            {
                SERIAL_DBG( "Reconnecting to blynk.. " );
                SERIAL_DBG( "Blynk connected: %d", Blynk.connected() );
                if ( !_blynkWifiClient.connected() )
                {
                    connectBlynk();
                    return;
                }

                //FIXME: add exit after n-unsuccesfull tries.
                Blynk.connect( 4000 );
                SERIAL_DBG( "Blynk connected: %d", Blynk.connected() );
            }
        }
    }

    if ( WiFi.status() != WL_CONNECTED && online )
    {
        int uptime = millis() / 1000;
        
        if ( !wifilost_flag )
        {
            wifilost_timer_start = uptime;
            wifilost_flag = true;
        }
        if ( ( ( uptime - wifilost_timer_start ) > wifilost_timer_max ) && wifilost_flag )
        {
            SERIAL_ERR( "WiFi connection lost, restarting.." );
            wifilost_flag = false;
            ESP.restart();
        }
    }

    timer.run();
    ESP.wdtFeed();

    while ( Serial.available() > 0 )
    {
        if ( Serial.read() == ' ' )
        {
            SayHello();
        }
    }
}
