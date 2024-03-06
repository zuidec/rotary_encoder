/*
 *	encoder.cpp
 *  Class to interface with rotary encoders using the Arduino framework	
 *
 *	Created by zuidec on 02/15/24
 */

#include "rotary_encoder.h"
#include "esp32-hal-gpio.h"
#include "esp32-hal.h"
#include "esp_bit_defs.h"
#include "pins_arduino.h"
#include <Arduino.h>
#include <sys/_stdint.h>

uint8_t RotaryEncoder::isr_instance_flag = 0;
RotaryEncoder *RotaryEncoder::encoderISRInstances[MAX_ENCODER_ISR];
RotaryEncoder *RotaryEncoder::switchISRInstances[MAX_ENCODER_ISR];

RotaryEncoder::RotaryEncoder(   uint8_t rotary_clk_pin, uint8_t rotary_dt_pin, 
                                uint8_t rotary_sw_pin, bool interrupt_enabled)  {
    CLK_PIN = rotary_clk_pin;
    DT_PIN  = rotary_dt_pin;
    SW_PIN  = rotary_sw_pin;

    pinMode(rotary_clk_pin, INPUT);
    pinMode(rotary_dt_pin, INPUT);
    pinMode(rotary_sw_pin, INPUT_PULLUP);
    
    last_encoder_position = (last_encoder_position & ~(1 << CLK_BIT)) | (digitalRead(CLK_PIN) << CLK_BIT);
    last_encoder_position = (last_encoder_position & ~(1 << DT_BIT)) | (digitalRead(DT_PIN) << DT_BIT);
    last_encoder_position = 0x00;
    current_encoder_position = last_encoder_position;
    last_switch_press = millis();
    switch_pressed = false;
    
    if(interrupt_enabled && (digitalPinToInterrupt(rotary_clk_pin) != NOT_AN_INTERRUPT) && (digitalPinToInterrupt(rotary_sw_pin) != NOT_AN_INTERRUPT))  {
        for(uint8_t i=0; i< MAX_ENCODER_ISR; i++)   {
            if(!(isr_instance_flag & _BV(i)))    {
                instance_id = i;
                encoderISRInstances[instance_id] = this;
                switchISRInstances[instance_id] = this;
                isr_instance_flag |= _BV(instance_id);
                break; 
            }   
        }

        static void((*encoderISRFunction[MAX_ENCODER_ISR])(void)) =     {
            encoderISR0, encoderISR1, encoderISR2, encoderISR3, 
            encoderISR4, encoderISR5, encoderISR6, encoderISR7
        };
        
        static void((*switchISRFunction[MAX_ENCODER_ISR])(void)) =     {
            switchISR0, switchISR1, switchISR2, switchISR3, 
            switchISR4, switchISR5, switchISR6, switchISR7
        };
        
        attachInterrupt(digitalPinToInterrupt(rotary_clk_pin), encoderISRFunction[instance_id], RISING);
        attachInterrupt(digitalPinToInterrupt(rotary_sw_pin), switchISRFunction[instance_id], FALLING);
    }

}

RotaryEncoder::~RotaryEncoder(void) {
    detachInterrupt(digitalPinToInterrupt(CLK_PIN));
    detachInterrupt(digitalPinToInterrupt(SW_PIN));
    isr_instance_flag &= ~_BV(instance_id);
}

RotaryEncoder::EncoderDirection RotaryEncoder::getDirection(void)  {

    if((current_encoder_position & (1 << CLK_BIT)) != (last_encoder_position & (1 << CLK_BIT))) {

        last_encoder_position = current_encoder_position;
        /*
         *  The rotary encoder when going CW causes the following:
         *  CLK_HIGH -> DT_HIGH -> CLK_LOW -> DT_LOW
         *  
         *  CCW rotation is the following:
         *  DT_HIGH -> CLK_HIGH -> DT_LOW -> CLK_LOW
         *
         *  By only calculating direction when CLK_BIT is changed, there are
         *  only 4 combinations of CLK and DT:
         *  CLK 1:  DT = 1 -> CCW   (0x03)
         *          DT = 0 -> CW    (0x02)
         *  CLK 0:  DT = 1 -> CW    (0x01)
         *          DT = 0 -> CCW   (0x00)
         *
         */

        switch(current_encoder_position)    {
            case 0x03:
                return ENCODER_CCW;
                break;
            case 0x02:
                return ENCODER_CW;
                break;
            case 0x01:
                return ENCODER_CW;
                break;
            case 0x00:
                return ENCODER_CCW;
            case 0xff:
                return ENCODER_TEST;
                break;
        }
        
    }
    
    return ENCODER_NOMOVE;

}

bool RotaryEncoder::isAvailable(void) {

    if((current_encoder_position & ( 1 << CLK_BIT)) != (last_encoder_position & ( 1 << CLK_BIT))) {
        return true;
    }
    else    {
        return false;
    }
}

void RotaryEncoder::instanceEncoderISR(void)  {

    current_encoder_position = ((current_encoder_position & ~(1 << CLK_BIT)) | (digitalRead(CLK_PIN) << CLK_BIT));
    current_encoder_position = ((current_encoder_position  & ~(1 << DT_BIT)) | (digitalRead(DT_PIN) << DT_BIT));
}

void RotaryEncoder::instanceSwitchISR(void)   {

    if(!digitalRead(SW_PIN) && (millis() - last_switch_press > SW_TIMEOUT_MS))  {
        switch_pressed = true;
        last_switch_press = millis();
    }
}

void RotaryEncoder::encoderISR0(void)   { RotaryEncoder::encoderISRInstances[0]->instanceEncoderISR(); }
void RotaryEncoder::switchISR0(void)    { RotaryEncoder::switchISRInstances[0]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR1(void)   { RotaryEncoder::encoderISRInstances[1]->instanceEncoderISR(); }
void RotaryEncoder::switchISR1(void)    { RotaryEncoder::switchISRInstances[1]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR2(void)   { RotaryEncoder::encoderISRInstances[2]->instanceEncoderISR(); }
void RotaryEncoder::switchISR2(void)    { RotaryEncoder::switchISRInstances[2]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR3(void)   { RotaryEncoder::encoderISRInstances[3]->instanceEncoderISR(); }
void RotaryEncoder::switchISR3(void)    { RotaryEncoder::switchISRInstances[3]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR4(void)   { RotaryEncoder::encoderISRInstances[4]->instanceEncoderISR(); }
void RotaryEncoder::switchISR4(void)    { RotaryEncoder::switchISRInstances[4]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR5(void)   { RotaryEncoder::encoderISRInstances[5]->instanceEncoderISR(); }
void RotaryEncoder::switchISR5(void)    { RotaryEncoder::switchISRInstances[5]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR6(void)   { RotaryEncoder::encoderISRInstances[6]->instanceEncoderISR(); }
void RotaryEncoder::switchISR6(void)    { RotaryEncoder::switchISRInstances[6]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR7(void)   { RotaryEncoder::encoderISRInstances[7]->instanceEncoderISR(); }
void RotaryEncoder::switchISR7(void)    { RotaryEncoder::switchISRInstances[7]->instanceSwitchISR(); }
