/*  OpenWindAir Smart CO2 sensor.
 *  Based on: ESP8266, MH-Z19, AM2302, Blynk and MQTT.
 *  Created in Arduino IDE.
 *  For more details please visit http://openwind.ru
 *  Contact us: hello@openwind.ru
 */

/*
 * 07.2019 dmitriev.dd@hotmail.com
 */

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <FS.h>
#include <string.h>
#include <MHZ19.h>
#include <vector>

//blynk
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>

//RTC Time
#include <NTPClient.h>
#include <WiFiUdp.h>

// https://github.com/plerup/espsoftwareserial
#include <SoftwareSerial.h>

// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>

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

#define SW_VERSION          "0.1.1"

#define BLYNK_GREEN         "#23C48E"
#define BLYNK_BLUE          "#04C0F8"
#define BLYNK_YELLOW        "#ED9D00"
#define BLYNK_RED           "#D3435C"
#define BLYNK_DARK_BLUE     "#5F7CD8"

/* PIN definitions */
#define DHTPIN              (12)
#define DHTTYPE             (DHT22)   // DHT 22  (AM2302), AM2321

#define LED_R_PIN           (15)
#define LED_G_PIN           (14)
#define LED_Y_PIN           (16)
#define ADC_PIN             (A0)
#define BUTTON_S1_PIN       (10)
#define BUTTON_S2_PIN       (0)
#define RELAY_PIN           (15)

/* Sketch parameters */
#define FILTER_POINTS       (4)
#define CO2_WARNING_SET     (1000)
#define CO2_WARNING_RESET   (700)

#define TIMER_SEND_UPTIME   (5000L)
#define TIMER_NOTIFY        (30000L)
#define TIMER_READ_MHZ19    (5000L)
#define TIMER_READ_DHT22    (20000L)
#define TIMER_READ_ADC      (60000L)
#define TIMER_SEND_RESULTS  (10000L)

#define LED_BRIGHT          (300)
#define LED_DIMMED          (20)

#define CO2_LEVEL_AVERAGE   (700)
#define CO2_LEVEL_POOR      (1200)

/* -------------------------
 *      Code starts here
 * ------------------------- */ 

DHT dht(DHTPIN, DHTTYPE);
Ticker ticker;
SoftwareSerial co2Serial(5, 4, false, 256);
BlynkTimer timer;
MHZ19 mhz19;

WiFiUDP ntpUDP;
NTPClient timeClient( ntpUDP, 10800 );  // +3 hours time offset

typedef enum 
{
    Led_None     = 0,
    Led_Green    = 1 << 0,
    Led_Yellow   = 1 << 1,
    Led_Red      = 1 << 2,
} LedColors;

char Hostname[ 32 ] = "FreshWindAir";

char blynk_token[ 34 ];
char blynk_server[ 40 ];
char blynk_port[ 6 ];

int ledXState = LED_BRIGHT;

float h = 0;
float t = 0;
float f = 0;
float hi = 0;   // heat index

std::vector<int> ppm_values( FILTER_POINTS, 0 );
int average_ppm_sum;
int average_ppm_prev;
int average_ppm_diff;

int adcvalue;

int ppm;

int co2_limit = 5; //allowed value of CO2 limit 2, 5 (2k, 5k). 5k default
bool co2_limit_flag = false;

float temp_correction = 3; // default enabled for internal DHT sensor. -3C -1F

bool notify_flag = false; //send notify to user if true
bool notify_flag_beep = false; //beep works if true
int notify_timer_start; //not allow to send notification too often.
int notify_timer_max = 600; //interval of notify 10 min by default

bool shouldSaveConfig = false; //flag for saving data
bool online = true;

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

static void setLeds( int colors )
{
    int newGreenState = 0;
    int newYellowState = 0;
    int newRedState = 0;

    if ( colors & Led_Green ) 
    {
        newGreenState = ledXState;
    }
    if ( colors & Led_Yellow ) 
    {
        newYellowState = ledXState;
    }
    if ( colors & Led_Red )
    {
        newRedState = ledXState;
    }
    
    analogWrite( LED_G_PIN, newGreenState );
    analogWrite( LED_Y_PIN, newYellowState );
    analogWrite( LED_R_PIN, newRedState );
}

static void restartMcu()
{
    terminal.print( "\n\rRestart in 3..2..1.." );
    terminal.flush();
    Serial.println( "\n\rRestart in 3..2..1.." );

    setLeds( ( LedColors ) Led_Green | Led_Yellow | Led_Red );
    
    ESP.restart();
}

static void resetWifiSettings() 
{
    terminal.print( "\n\rReset WiFi settings in 3..2..1.." );
    terminal.flush();
    Serial.println( "\n\rReset WiFi settings in 3..2..1.." );

    // wifiManager.resetSettings(); 
    restartMcu();
}

static void formatFlash() 
{
    terminal.print( "\n\rFormat flash in 3..2..1.." );
    terminal.flush();
    Serial.println( "\n\rFormat flash in 3..2..1.." );

    SPIFFS.format();
    restartMcu();
}

static int getAverage( std::vector<int> v ) 
{
    int sum = 0;
    for ( size_t i = 0; i < v.size(); ++i ) 
    {
        sum += v[ i ];
    }
    return sum / v.size();
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

BLYNK_WRITE( V105 )
{
    int v105 = param.asInt();

    if ( v105 != 0 )
    {
        notify_flag_beep = true;
    }
    else
    {
        notify_flag_beep = false;
    }
}

BLYNK_WRITE( V107 )
{
    float v107 = param.asInt();

    temp_correction = v107;
    Serial.print( "\r\ntemp_correction (C): " );
    Serial.print( temp_correction );
    terminal.print( "\r\ntemp_correction (C): " );
    terminal.print( temp_correction );
    terminal.flush();
}

BLYNK_WRITE( V110 )
{
    ledXState = param.asInt();

    if ( ledXState < 100 )
    {
        ledXState = 100;
    }

    if ( ledXState > 1000 )
    {
        ledXState = 1000;
    }

    Serial.print( "\r\nledXState: " );
    Serial.print( ledXState );
}

void tick()
{
    //toggle state
    int state = digitalRead( LED_R_PIN );  // get the current state of Pin
    digitalWrite( LED_R_PIN, !state );     // set Pin to the opposite state
}

// toggle LED state. for future use
void led_toggle_r()
{
    int state = digitalRead( LED_R_PIN );  // get the current state of Pin
    digitalWrite( LED_R_PIN, !state );     // set Pin to the opposite state
}

void led_toggle_g()
{
    int state = digitalRead( LED_G_PIN );  // get the current state of Pin
    digitalWrite( LED_G_PIN, !state );     // set Pin to the opposite state
}

void led_toggle_y()
{
    int state = digitalRead( LED_Y_PIN );  // get the current state of Pin
    digitalWrite( LED_Y_PIN, !state );     // set Pin to the opposite state
}

/* -------------------------
 *      WiFi Manager cb
 * ------------------------- */ 

void configModeCallback( WiFiManager *myWiFiManager )
{
    //gets called when WiFiManager enters configuration mode
    Serial.println( "Entered config mode" );
    Serial.println( WiFi.softAPIP() );
    //if you used auto generated SSID, print it
    Serial.println( myWiFiManager->getConfigPortalSSID() );
    //entered config mode, make led toggle faster
    ticker.attach( 0.2, led_toggle_r );
}

void saveConfigCallback()
{  
    //callback notifying us of the need to save config
    Serial.println( "Should save config" );
    shouldSaveConfig = true;
    ticker.attach( 0.2, led_toggle_r );  // led toggle faster
}

/* -------------------------
 *      Main functions
 * ------------------------- */ 

void notify()
{
//    if ( !notify_flag && average_ppm_max > 400 && average_ppm_sum >= average_ppm_max )
//    {
//        notify_flag = !notify_flag;
//        notify_timer_start = uptime;
//    }
//
//    if ( notify_flag && average_ppm_max > average_ppm_sum )
//    {
//        notify_flag = !notify_flag;
//    }
//
//    if ( notify_flag && ( ( uptime - notify_timer_start ) > notify_timer_max ) )
//    {
//
//        Blynk.notify( String( "CO2 level > " ) + average_ppm_sum + ". Please Open Window." );
//        terminal.print( "\n\rSending notify to phone. " );
//        terminal.print( "ppm > " );
//        terminal.print( average_ppm_sum );
//        terminal.flush();
//        Serial.print( "\n\rSending notify to phone. " );
//        Serial.print( "\n\rCO2 level > " );
//        Serial.print( average_ppm_sum );
//
//        tones( 13, 1000, 50 );
//        delay( 50 );
//        tones( 13, 1000, 50 );
//        delay( 50 );
//        tones( 13, 1000, 50 );
//
//        notify_flag = false;
//    }
//
//    if ( notify_flag )
//    {
//        terminal.print( "\n\rNotify in: " );
//        terminal.print( notify_timer_max + notify_timer_start - uptime );
//        terminal.print( " seconds" );
//        terminal.flush();
//
//        Serial.print( "\n\rNotify in: " );
//        Serial.print( notify_timer_max + notify_timer_start - uptime );
//        Serial.print( " seconds" );
//    }
}

void readMHZ19()
{
    int i = 0;
    ppm = -1;
    Serial.print( "\n\rReading MHZ19 sensor:" );
    while ( i < 5 && ppm == -1 )
    {
        delay( i * 50 );
        ppm = mhz19.getCO2();
        i++;
    }
    if ( ppm == -1 )
    {
        led2.on();
        led2.setColor( BLYNK_YELLOW );
        Serial.print( " failed" );

        terminal.print( "\n\r[Error] Can't read CO2!" );
        terminal.flush();

        return;
    }
    else
    {
        led2.on();
        led2.setColor( BLYNK_GREEN );
        Serial.print( " ok" );
    }
    
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
        setLeds( Led_Green );

        led1.on();
        led1.setColor( BLYNK_GREEN );
        led2.on();
        led2.setColor( BLYNK_GREEN );
    }
    else if ( average_ppm_sum < CO2_LEVEL_POOR )
    {
        setLeds( Led_Yellow );

        led1.on();
        led1.setColor( BLYNK_YELLOW );
        led2.on();
        led2.setColor( BLYNK_GREEN );
    }
    else 
    {
        setLeds( Led_Red );

        led1.on();
        led1.setColor( BLYNK_RED );
        led2.on();
        led2.setColor( BLYNK_GREEN );
    }
}

void readDHT22()
{
    bool DHTreadOK = false;
    int i = 0;
    Serial.print( "\n\rReading DHT22 sensor:" );
    while ( i < 5 && !DHTreadOK )
    {
        delay( i * 75 );
        h = dht.readHumidity();
        t = dht.readTemperature();
        f = dht.readTemperature( true ); // Read temperature as Fahrenheit (isFahrenheit = true)
        hi = dht.computeHeatIndex( t, h, false );        

        if ( isnan( h ) || isnan( t ) || isnan( f ) )
        {
            Serial.print( "!" );
            i++;
        }
        else
        {
            DHTreadOK = true;

            t = t - temp_correction;
            f = f - 1;
        }
    }
    if ( DHTreadOK )
    {
        led2.on();
        led2.setColor( BLYNK_GREEN );
        Serial.print( " ok" );
    }
    else
    {
        led2.on();
        led2.setColor( BLYNK_RED );
        Serial.print( " failed" );

        terminal.print( "\n\r[Error] Can't read CO2!" );
        terminal.flush();
    }
}

void readADC()
{
    adcvalue = analogRead( ADC_PIN );
    if ( Blynk.connected() )
    {
        Blynk.virtualWrite( V5, adcvalue );
    }
}

void SayHello()
{
    Serial.print( "\n\r====== SYSTEM-STATUS ================================" );
    Serial.print( "\n\rDevice name: " );
    Serial.print( Hostname );
    Serial.print( "\r\nSoftware version: " );
    Serial.print( SW_VERSION );
    Serial.print( "\r\nFreeHeap: " );
    Serial.print( ESP.getFreeHeap() );
    Serial.print( "\r\nChipId: " ); //ESP8266 chip IDE, int 32bit
    Serial.print( ESP.getChipId() );
    Serial.print( "\r\nFlashChipId: " ); //flash chip ID, int 32bit
    Serial.print( ESP.getFlashChipId() );
    Serial.print( "\r\nFlashChipSize: " );
    Serial.print( ESP.getFlashChipSize() );
    Serial.print( "\r\nFlashChipSpeed: " );
    Serial.print( ESP.getFlashChipSpeed() );
    Serial.print( "\r\nCycleCount: " ); //unsigned 32-bit
    Serial.print( ESP.getCycleCount() );
    Serial.print( "\r\nTime: " );
    Serial.print( timeClient.getFormattedTime() );
    Serial.print( "\r\nUpTime: " );
    Serial.print( millis() / 1000 );
    Serial.print( "\n\r====== BLYNK-STATUS =================================" );
    Serial.print( "\n\rBlynk token: " );
    Serial.print( blynk_token );
    Serial.print( "\n\rBlynk connected: " );
    Serial.print( Blynk.connected() );
//    if ( average_ppm_max == 400 )
//    {
//        Serial.print( "\r\nNotify level: disabled" );
//    }
//    else
//    {
//        Serial.print( "\r\nNotify level: " );
//        Serial.print( average_ppm_max );
//    }
    Serial.print( "\r\nBeep: " );
    Serial.print( notify_flag_beep );
    Serial.print( "\r\nCO2 limit: " );
    Serial.print( co2_limit * 1000 );
    Serial.print( "\r\nTemperature correction: " );
    Serial.print( temp_correction );
    Serial.print( " C" );
    Serial.print( "\r\nledXState: " );
    Serial.print( ledXState );
    Serial.print( "\n\r====== NETWORK-STATUS ===============================" );
    Serial.print( "\n\rWiFi network: " );
    Serial.print( WiFi.SSID() );
    Serial.print( "\n\rWiFi status: " );
    Serial.print( WiFi.status() );
    Serial.print( "\r\nRSSI: " );
    Serial.print( WiFi.RSSI() );
    Serial.print( "\n\rMAC: " );
    Serial.print( WiFi.macAddress() );
    Serial.print( "\n\rIP: " );
    Serial.print( WiFi.localIP() );
    Serial.print( "\n\rOnline: " );
    Serial.print( online );
    Serial.println( "====== END-of-STATUS ================================" );

}

void tones( uint8_t _pin, unsigned int frequency, unsigned long duration )
{
    if ( notify_flag_beep )
    {
        pinMode( _pin, OUTPUT );
        analogWriteFreq( frequency );
        analogWrite( _pin, 500 );
        delay( duration );
        analogWrite( _pin, 0 );
    }
}

String getFormattedUptime() 
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

    return daysStr + "d " + hoursStr + "h " + minuteStr + "m " + secondStr + "s";
}

void sendUptime()
{
    if ( Blynk.connected() )
    {
        Blynk.virtualWrite( V99, millis() / 1000 );
    }
}

void sendResults()
{
    Serial.println( "\n\r===================================================" );

    Serial.print( "\n\rUpTime: " );
    Serial.print( getFormattedUptime() );

    Serial.print( "\n\rTime: " );
    Serial.print( timeClient.getFormattedTime() );

    terminal.print( "\n\n\rUpTime: " );
    terminal.print( getFormattedUptime() );
    
    /* DHT info */
    Blynk.virtualWrite( V1, h );
    Blynk.virtualWrite( V2, t );
    Blynk.virtualWrite( V3, f );
    Blynk.virtualWrite( V7, hi );

    terminal.print( "\n\rHumidity: " );
    terminal.print( h );
    terminal.print( "%" );
    terminal.print( "\n\rTemperature: " );
    terminal.print( t );
    terminal.print( " C" );
    terminal.print( "\n\rFeels like: " );
    terminal.print( hi );
    terminal.print( " C" );

    Serial.print( "\n\rHumidity: " );
    Serial.print( h );
    Serial.print( "%" );
    Serial.print( "\n\rTemperature: " );
    Serial.print( t );
    Serial.print( " C \\ " );
    Serial.print( f );
    Serial.print( " F" );
    Serial.print( "\n\rFeels like: " );
    Serial.print( hi );
    Serial.print( " C" );

    /* MHZ-19 info */
    Blynk.virtualWrite( V4, average_ppm_sum );
    Blynk.virtualWrite( V6, average_ppm_diff );

    /* We should compare with previous sent value */
    average_ppm_prev = average_ppm_sum;

    terminal.print( "\n\rC02 average: " );
    terminal.print( average_ppm_sum );
    terminal.print( " ppm" );
    terminal.print( " (diff: " );
    terminal.print( average_ppm_diff );
    terminal.print( " ppm)" );

    Serial.print( "\n\rC02: " );
    Serial.print( ppm );
    Serial.print( " ppm" );
    Serial.print( "\n\rC02 average: " );
    Serial.print( average_ppm_sum );
    Serial.print( " ppm" );

    /* ADC info */
    Serial.print( "\n\rADC: " );
    Serial.print( adcvalue );

    terminal.flush();

    Serial.println( "\n\r===================================================" );
}

// Setup
void setup()
{
    Serial.begin( 115200 );
    delay( 2000 );

    co2Serial.begin( 9600 );
    mhz19.begin( co2Serial );
    mhz19.recoveryReset();
    mhz19.autoCalibration();
    delay( 2000 );

    dht.begin();

    pinMode( BUTTON_S1_PIN, INPUT );
    pinMode( BUTTON_S2_PIN, INPUT );
    //pinMode( RELAY_PIN, OUTPUT );
    pinMode( ADC_PIN, INPUT );
    pinMode( LED_R_PIN, OUTPUT );
    pinMode( LED_G_PIN, OUTPUT );
    pinMode( LED_Y_PIN, OUTPUT );

    delay( 2000 );

    tones( 13, 1000, 100 );

    // start ticker with 0.5 because we start in AP mode and try to connect
    ticker.attach( 0.6, led_toggle_r ); //tick

    // Check flash size   
    if ( ESP.getFlashChipRealSize() == ESP.getFlashChipSize() )
    {
        Serial.print( "flash correctly configured, SPIFFS starts, IDE size: " );
        Serial.print( ESP.getFlashChipSize() );
        Serial.print( ", match real size: " );
        Serial.println( ESP.getFlashChipRealSize() );
        Serial.println();
    }
    else
    {
        Serial.print( "flash incorrectly configured, SPIFFS cannot start, IDE size: " );
        Serial.print( ESP.getFlashChipSize() );
        Serial.print( ", real size: " );
        Serial.println( ESP.getFlashChipRealSize() );
    }

    //read configuration from FS json
    Serial.println( "mounting FS..." );

    if ( SPIFFS.begin() )
    {
        Serial.println( "mounted file system" );
        if ( SPIFFS.exists( "/config.json" ) )
        {
            //file exists, reading and loading
            Serial.println( "reading config file" );
            File configFile = SPIFFS.open( "/config.json", "r" );
            if ( configFile )
            {
                Serial.println( "opened config file" );
                size_t size = configFile.size();
                // Allocate a buffer to store contents of the file.
                std::unique_ptr<char[]> buf(new char[size]);

                configFile.readBytes( buf.get(), size );
                DynamicJsonBuffer jsonBuffer;
                JsonObject & json = jsonBuffer.parseObject( buf.get() );
                json.printTo( Serial );
                if ( json.success() )
                {
                    Serial.println( "\nparsed json" );

                    strcpy( blynk_token, json[ "blynk_token" ] );
                }
                else
                {
                    Serial.println( "Failed to load json config" );
                }
            }
        }
    }
    else
    {
        Serial.println( "Failed to mount FS" );
    }
    //end read

    // The extra parameters to be configured (can be either global or just in the setup)
    // After connecting, parameter.getValue() will get you the configured value
    // id/name placeholder/prompt default length

    WiFiManagerParameter
    custom_blynk_token( "blynk", "blynk token", blynk_token, 33 );   // was 32 length ???

    //WiFiManager
    //Local intialization. Once its business is done, there is no need to keep it around
    WiFiManager wifiManager;

    // WiFi credentials will be reseted if button S1 will be pressed during boot
    int buttonS1State = digitalRead( BUTTON_S1_PIN );
    if ( buttonS1State == 0 )
    {
        Serial.println( "Reset Wi-Fi settings" );
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

    if ( !wifiManager.autoConnect( "FreshWind - tap to config" ) )
    {
        if ( blynk_token[ 0 ] != '\0' )
        {
            Serial.println( "Failed to go online for Blynk, restarting.." );
            delay( 2000 );
            ESP.restart();
        }
        else
        {
            Serial.println( "Failed to go online, offline mode activated" );
            online = false;
            tones( 13, 2000, 50 );
        }
    }

    ticker.detach();

    if ( online )
    {
        tones( 13, 1500, 30 );

        strcpy( blynk_token, custom_blynk_token.getValue() );    //read updated parameters

        if ( shouldSaveConfig )
        {      
            //save the custom parameters to FS
            Serial.println( "saving config" );
            DynamicJsonBuffer jsonBuffer;
            JsonObject & json = jsonBuffer.createObject();
            json[ "blynk_token" ] = blynk_token;

            File configFile = SPIFFS.open( "/config.json", "w" );
            if ( !configFile )
            {
                Serial.println( "Failed to open config file for writing" );
            }

            json.printTo( Serial );
            json.printTo( configFile );
            configFile.close();
            //end save

            delay( 1000 );
            Serial.println( "Restart ESP to apply new WiFi settings.." );
            ESP.restart();
        }

        Serial.print( "\n\rWiFi network: " );
        Serial.print( WiFi.SSID() );

        Serial.print( "\n\rWiFi status: " );
        Serial.print( WiFi.status() );

        Serial.print( "\n\rlocal ip: " );
        Serial.print( WiFi.localIP() );

        if ( blynk_token[ 0 ] != '\0' )
        {
            connectBlynk();
            Blynk.config( blynk_token );
            Blynk.connect();
            Serial.print( "\n\rblynk token: " );
            Serial.print( blynk_token );
        }
        else
        {
            Serial.print( "\n\rblynk auth token not set" );
        }

        Serial.print( "\n\rFreshWindAir is ready!" );

        Serial.print( "\n\rStart time sync client\n\r" );
        timeClient.begin();
    }

    timer.setInterval( TIMER_SEND_UPTIME, sendUptime );
    timer.setInterval( TIMER_NOTIFY, notify );
    timer.setInterval( TIMER_READ_MHZ19, readMHZ19 );
    timer.setInterval( TIMER_READ_DHT22, readDHT22 );
    timer.setInterval( TIMER_READ_ADC, readADC );
    timer.setInterval( TIMER_SEND_RESULTS, sendResults );
    
    // Serial.setDebugOutput( true );

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
                Serial.print( "\n\rReconnecting to blynk.. " );
                Serial.print( Blynk.connected() );
                if ( !_blynkWifiClient.connected() )
                {
                    connectBlynk();
                    return;
                }

                //FIXME: add exit after n-unsuccesfull tries.
                Blynk.connect( 4000 );
                Serial.print( Blynk.connected() );
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
            Serial.print( "\n\rWiFi connection lost, restarting.." );
            wifilost_flag = false;
            ESP.restart();
        }
    }

    timer.run();
    ESP.wdtFeed();

    /* Dim led indication for evening */
    if ( timeClient.getHours() >= 22 || timeClient.getHours() <= 8 )
    {
        ledXState = LED_DIMMED;
    }
    else 
    {
        ledXState = LED_BRIGHT;
    }

    /* Buttons handling */
    int buttonS1State = digitalRead( BUTTON_S1_PIN );
    int buttonS2State = digitalRead( BUTTON_S2_PIN );

    static unsigned long lastCalibrationTime = 0;
    if ( buttonS2State == 0 )
    {
        if ( millis() - lastCalibrationTime > 10000 )
        {       
            Serial.print( "\n\rMHZ-19 recalibrate zero now..." );
            terminal.print( "\n\rMHZ-19 recalibrate zero now..." );
            terminal.flush();
        
            mhz19.calibrateZero();   

            lastCalibrationTime = millis();
        }
    }

    while ( Serial.available() > 0 )
    {
        if ( Serial.read() == '\r' || Serial.read() == '\n' )
        {
            SayHello();
            tones( 13, 1000, 100 );
        }
    }
}
