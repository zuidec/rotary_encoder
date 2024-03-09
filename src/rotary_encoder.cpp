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
RotaryEncoder *RotaryEncoder::ISRInstances[MAX_ENCODER_ISR];

RotaryEncoder::RotaryEncoder(   uint8_t rotary_clk_pin, uint8_t rotary_dt_pin, 
                                uint8_t rotary_sw_pin, bool interrupt_enabled)  {
    CLK_PIN = rotary_clk_pin;
    DT_PIN  = rotary_dt_pin;
    SW_PIN  = rotary_sw_pin;

    pinMode(CLK_PIN, INPUT);
    pinMode(DT_PIN,  INPUT);
    pinMode(SW_PIN, INPUT_PULLUP);
    digitalWrite(CLK_PIN, HIGH); 
    digitalWrite(DT_PIN, HIGH); 
    
    encoder_state = ENCODER_START_STATE;
    last_encoder_position = DIR_NO_MOVE;
    last_switch_press = millis();
    switch_pressed = false;
    
    if(interrupt_enabled && (digitalPinToInterrupt(CLK_PIN) != NOT_AN_INTERRUPT) && (digitalPinToInterrupt(SW_PIN) != NOT_AN_INTERRUPT))  {
        for(uint8_t i=0; i< MAX_ENCODER_ISR; i++)   {
            if(!(isr_instance_flag & _BV(i)))    {
                instance_id = i;
                ISRInstances[instance_id] = this;
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
        
        attachInterrupt(digitalPinToInterrupt(CLK_PIN), encoderISRFunction[instance_id], CHANGE);
        attachInterrupt(digitalPinToInterrupt(DT_PIN), encoderISRFunction[instance_id], CHANGE);
        attachInterrupt(digitalPinToInterrupt(SW_PIN), switchISRFunction[instance_id], FALLING);
    }

}

RotaryEncoder::~RotaryEncoder(void) {
    detachInterrupt(digitalPinToInterrupt(CLK_PIN));
    detachInterrupt(digitalPinToInterrupt(DT_PIN));
    detachInterrupt(digitalPinToInterrupt(SW_PIN));
    isr_instance_flag &= ~_BV(instance_id);
}

uint8_t RotaryEncoder::getDirection(void)   {
    uint8_t ret = last_encoder_position;
    last_encoder_position = DIR_NO_MOVE;
    return ret;
}

bool RotaryEncoder::switchPressed(void) {
    bool ret = switch_pressed;
    switch_pressed = false;
    return ret;
}
bool RotaryEncoder::isAvailable(void) {

    if(last_encoder_position != DIR_NO_MOVE) {
        return true;
    }
    else    {
        return false;
    }
}

void RotaryEncoder::instanceEncoderISR(void)  {

    uint8_t current_encoder_position = (digitalRead(CLK_PIN) << 1) | digitalRead(DT_PIN);
    encoder_state = state_table[encoder_state & 0x0F][current_encoder_position];
    switch(encoder_state&0x30)  {
        case DIR_CW:
            last_encoder_position = DIR_CW;
            break;
        case DIR_CCW:
            last_encoder_position = DIR_CCW;
        default:
            break;
    }
}

void RotaryEncoder::instanceSwitchISR(void)   {

    if(!digitalRead(SW_PIN) && (millis() - last_switch_press > SW_TIMEOUT_MS))  {
        switch_pressed = true;
        last_switch_press = millis();
    }
}

void RotaryEncoder::encoderISR0(void)   { RotaryEncoder::ISRInstances[0]->instanceEncoderISR(); }
void RotaryEncoder::switchISR0(void)    { RotaryEncoder::ISRInstances[0]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR1(void)   { RotaryEncoder::ISRInstances[1]->instanceEncoderISR(); }
void RotaryEncoder::switchISR1(void)    { RotaryEncoder::ISRInstances[1]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR2(void)   { RotaryEncoder::ISRInstances[2]->instanceEncoderISR(); }
void RotaryEncoder::switchISR2(void)    { RotaryEncoder::ISRInstances[2]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR3(void)   { RotaryEncoder::ISRInstances[3]->instanceEncoderISR(); }
void RotaryEncoder::switchISR3(void)    { RotaryEncoder::ISRInstances[3]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR4(void)   { RotaryEncoder::ISRInstances[4]->instanceEncoderISR(); }
void RotaryEncoder::switchISR4(void)    { RotaryEncoder::ISRInstances[4]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR5(void)   { RotaryEncoder::ISRInstances[5]->instanceEncoderISR(); }
void RotaryEncoder::switchISR5(void)    { RotaryEncoder::ISRInstances[5]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR6(void)   { RotaryEncoder::ISRInstances[6]->instanceEncoderISR(); }
void RotaryEncoder::switchISR6(void)    { RotaryEncoder::ISRInstances[6]->instanceSwitchISR(); }
void RotaryEncoder::encoderISR7(void)   { RotaryEncoder::ISRInstances[7]->instanceEncoderISR(); }
void RotaryEncoder::switchISR7(void)    { RotaryEncoder::ISRInstances[7]->instanceSwitchISR(); }
