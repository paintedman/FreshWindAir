#include "sensors.h"

FreshWindSensors::FreshWindSensors() {
    ppmValues = new std::vector<int>( FILTER_POINTS, 0 );
    co2Serial = new SoftwareSerial( MHZ_RX_PIN, MHZ_TX_PIN, false );
    dht = new DHT( DHTPIN, DHTTYPE );
}

void FreshWindSensors::initMhz19() {
    co2Serial->begin( 9600 );
    co2Serial->write( MHZ19Cmd_setRange2k, 9 );
    co2Serial->write( MHZ19Cmd_setAbcOn, 9 );

    /* Wait MHZ-19 to become ready */
    delay( 5000 );
}

void FreshWindSensors::initDht22() {
    dht->begin();
}

int FreshWindSensors::readMhz19() {
    byte response[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    /* Clear RX buffer before sending command */
    while ( co2Serial->available() > 0 ) {
        co2Serial->read();
    } 

    co2Serial->write( MHZ19Cmd_getCO2, 9 ); 
    co2Serial->flush();

    int len = co2Serial->readBytes( response, 9 );
    if ( len != 9 ) {
        return -1;
    }
    if ( response[0] != 0xFF ) {
        return -2;
    }
    if ( response[1] != 0x85 ) {
        return -3;
    }
    byte crc = getCRC( response, 9 );
    if ( response[ 8 ] != crc ) {
        return -4;
    }
    return ( response[ 4 ] << 8 ) | response[ 5 ];
}

void FreshWindSensors::updateMhz19() {  
    int ppm = readMhz19();
    if ( ppm < 0 ) {
        mhz19Errors++;
    }
    else {
        ppmValues->erase( ppmValues->begin() );
        ppmValues->push_back( ppm );
        avgPpm = std::accumulate( ppmValues->begin(), ppmValues->end(), 0LL ) / ppmValues->size();
    }
}

void FreshWindSensors::updateDht22() {
    float local_h = dht->readHumidity();
    float local_t = dht->readTemperature();
    float local_f = dht->readTemperature( true ); // Read temperature as Fahrenheit (isFahrenheit = true)   

    if ( isnan( local_h ) || isnan( local_t ) || isnan( local_f ) ) {
        dht22Errors++;
    }
    else {
        humidity = local_h;
        temperature = local_t - temp_correction;
        temperatureF = local_f - 1;
        heatIndex = dht->computeHeatIndex( temperature, humidity, false );
    }
}

void FreshWindSensors::setMhz19Abc( bool state ) {
    if ( state ) {
        co2Serial->write( MHZ19Cmd_setAbcOn, 9 );
    } else {
        co2Serial->write( MHZ19Cmd_setAbcOff, 9 );
    }
}