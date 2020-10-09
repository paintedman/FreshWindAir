#ifndef SENSORS_H
#define SENSORS_H

#include <vector>
// https://github.com/adafruit/DHT-sensor-library
#include <DHT.h>
// https://github.com/plerup/espsoftwareserial
#include <SoftwareSerial.h>

#define FILTER_POINTS       (6)
#define MHZ_RX_PIN          (5)
#define MHZ_TX_PIN          (4)
#define DHTPIN              (12)
#define DHTTYPE             (DHT22)   // DHT 22  (AM2302), AM2321

static byte getCRC( byte inBytes[], int size ) {
    byte x = 0, CRC = 0;
    for ( x = 1; x < size - 1; x++ ) {
        CRC += inBytes[ x ];
    }
    CRC = 255 - CRC;
    CRC++;

    return CRC;
}

class FreshWindSensors {
    public:
        FreshWindSensors();

        void clearErrors() {
            mhz19Errors = 0;
            dht22Errors = 0;
        }

        void initMhz19();
        void initDht22();

        void updateMhz19();
        void updateDht22();

        void setMhz19Abc( bool state );

        int getAveragePpm() {
            return avgPpm;
        };
        int getPpmDiff() {
            return avgPpmDiff;
        }
        float getHumidity() {
            return humidity;
        }
        float getTemperature() {
            return temperature;
        }
        float getTemperatureF() {
            return temperatureF;
        }
        float getHeatIndex() {
            return heatIndex;
        }
        uint32_t getMhz19Errors() {
            return mhz19Errors;
        }
        uint32_t getDht22Errors() {
            return dht22Errors;
        }

    private:
        SoftwareSerial* co2Serial = nullptr;
        DHT* dht = nullptr;

        int readMhz19();

        uint32_t mhz19Errors = 0;
        uint32_t dht22Errors = 0;

        // DHT22 values
        float temp_correction = 3.0;    // default correction for DHT22
        float humidity = 0.0;
        float temperature = 0.0;
        float temperatureF = 0.0;
        float heatIndex = 0.0;  

        // MHZ-19 values
        int ppm = 0;
        int avgPpm = 0;
        int avgPpmPrev = 0;
        int avgPpmDiff = 0;
        std::vector<int>* ppmValues;

        // command to ask for data
        const byte MHZ19Cmd_getCO2[9] = { 0xFF, 0x01, 0x85, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7A };
        const byte MHZ19Cmd_setRange2k[9] = { 0xFF, 0x01, 0x99, 0x00, 0x00, 0x00, 0x07, 0xD0, 0x8F };
        const byte MHZ19Cmd_setAbcOff[9] = { 0xFF, 0x01, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86 };
        const byte MHZ19Cmd_setAbcOn[9] = { 0xFF, 0x01, 0x79, 0xA0, 0x00, 0x00, 0x00, 0x00, 0xE6 };
};

#endif // SENSORS_H