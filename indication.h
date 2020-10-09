#ifndef INDICATION_H
#define INDICATION_H

#define LED_BRIGHT          (300)
#define LED_DIMMED          (20)
#define LED_OFF             (0)

#define LED_R_PIN           (15)
#define LED_G_PIN           (14)
#define LED_Y_PIN           (16)

//LED ticker
#include <Ticker.h>

#include <Arduino.h>

enum LedColor {
    Green,
    Yellow,
    Red,
};

class Indication {
    public:
        Indication();

        void clear();
        void setIndication( bool green, bool yellow, bool red );
        void setTickerFor( LedColor color, float rate = 0.5 );
        void setBrightness( int brightness );

    private:
        void updateIndication();

        Ticker* ticker;
        int brightness = LED_BRIGHT;

        bool stateGreen = false;
        bool stateYellow = false;
        bool stateRed = false;
};

#endif