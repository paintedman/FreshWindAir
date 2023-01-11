#include "indication.h"

Indication::Indication() {
    ticker = new Ticker();

    pinMode( LED_R_PIN, OUTPUT );
    pinMode( LED_G_PIN, OUTPUT );
    pinMode( LED_Y_PIN, OUTPUT );
}

void Indication::clear() {
    ticker->detach();
    stateGreen = false;
    stateYellow = false;
    stateRed = false;

    updateIndication();
}

void Indication::setBrightness( int value ) {
    if ( value < 100 ) {
        value = 0;
    }
    if ( value > 1000 ) {
        value = 1000;
    }
    brightness = value;

    updateIndication();
}

void Indication::setIndication( bool green, bool yellow, bool red ) {
    stateGreen = green;
    stateYellow = yellow;
    stateRed = red;

    updateIndication();
}

void Indication::setTickerFor( LedColor color, float rate ) {
    if ( color == LedColor::Green ) {
        ticker->attach( rate, []() {
            int state = digitalRead( LED_G_PIN );  // get the current state of Pin
            digitalWrite( LED_G_PIN, !state );     // set Pin to the opposite state
        } );
    } 
    if ( color == LedColor::Yellow ) {
        ticker->attach( rate, []() {
            int state = digitalRead( LED_Y_PIN );  // get the current state of Pin
            digitalWrite( LED_Y_PIN, !state );     // set Pin to the opposite state
        } );
    }
    if ( color == LedColor::Red ) {
        ticker->attach( rate, []() {
            int state = digitalRead( LED_R_PIN );  // get the current state of Pin
            digitalWrite( LED_R_PIN, !state );     // set Pin to the opposite state
        } );
    }
}

void Indication::updateIndication() {
    analogWrite( LED_G_PIN, stateGreen ? brightness : 0 );
    analogWrite( LED_Y_PIN, stateYellow ? brightness : 0 );
    analogWrite( LED_R_PIN, stateRed ? brightness : 0 );
}