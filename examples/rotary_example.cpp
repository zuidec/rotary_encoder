#include "Adafruit_SH110X.h"
#include "esp32-hal-gpio.h"
#include "pins_arduino.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "rotary_encoder.h"

#define ROTARY_CLK      5
#define ROTARY_DATA     4
#define ROTARY_SW       18
#define SCREEN_SDA      7
#define SCREEN_SCL      6
#define SCREEN_ADDR     (0x3C)
#define SCREEN_HEIGHT   128
#define SCREEN_WIDTH    128
#define OLED_RESET      -1
#define LOGO_HEIGHT     16
#define LOGO_WIDTH      16


int counter = 0;
String direction = "";
unsigned long last_button_press;

Adafruit_SH1107 display = Adafruit_SH1107(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET, 1000000, 1000000);
RotaryEncoder encoder = RotaryEncoder(ROTARY_CLK, ROTARY_DATA, ROTARY_SW, true);

static const unsigned char PROGMEM logo_bmp[] =
{ 0b00000000, 0b11000000,
  0b00000001, 0b11000000,
  0b00000001, 0b11000000,
  0b00000011, 0b11100000,
  0b11110011, 0b11100000,
  0b11111110, 0b11111000,
  0b01111110, 0b11111111,
  0b00110011, 0b10011111,
  0b00011111, 0b11111100,
  0b00001101, 0b01110000,
  0b00011011, 0b10100000,
  0b00111111, 0b11100000,
  0b00111111, 0b11110000,
  0b01111100, 0b11110000,
  0b01110000, 0b01110000,
  0b00000000, 0b00110000 
};

void init_display(void);
void update_display(void);

void setup()    {
    Serial.begin(115200);
    Wire.begin(SCREEN_SDA, SCREEN_SCL);

    delay(250);
    init_display();
    last_button_press = millis();
    Serial.println("Setup complete");
}

void loop() {
    
    if(encoder.isAvailable())   {
        switch(encoder.getDirection())  {
            case RotaryEncoder::ENCODER_CW:
                direction = "CW ";
                counter++;
                break;
            case RotaryEncoder::ENCODER_CCW:
                direction  = "CCW";
                counter--;
                break;
            case RotaryEncoder::ENCODER_TEST:
                direction = "GOOD INT";
                break;
            default:
                break;

        }
    }
    update_display();
}

void init_display(void) {

    display.begin(SCREEN_ADDR, true);
    display.display();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0,0);
    display.cp437(true);
    display.println("Ready to read encoder");
    display.display();
}

void update_display(void)  {

    display.clearDisplay();
    display.setCursor(0,1);
    display.print("Direction: ");
    display.println(direction);
    display.print("Counter: ");
    display.println(counter);
    if(millis() - last_button_press < 1500 && millis() > 2500) {
        display.println("Button pressed");
    }
    if(encoder.switch_pressed )  {
        last_button_press = millis();
        encoder.switch_pressed = false;
    }
    display.display();
    delay(1);
}
