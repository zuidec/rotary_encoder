/*
 *	encoder.h
 *  Class to interface with rotary encoders using the Arduino framework	
 *
 *	Created by zuidec on 02/15/24
 */

#ifndef ENCODER_H
#define ENCODER_H	// BEGIN ENCODER_H

/*
 *	Includes
 */

#include <stdint.h>
#include <sys/_stdint.h>


/*
 *	Defines
 */

#define CLK_BIT         (0x01)
#define DT_BIT          (0x00)
#define SW_TIMEOUT_MS   (100)
#define MAX_ENCODER_ISR (8)


/*
 *  Class
 */

class RotaryEncoder {
    public:
        enum EncoderDirection   {
            ENCODER_CCW, ENCODER_CW, ENCODER_NOMOVE, ENCODER_TEST
        };

        EncoderDirection getDirection(void);
        bool isAvailable(void);
        RotaryEncoder(uint8_t rotary_clk_pin, uint8_t rotary_dt_pin, 
            uint8_t rotary_sw_pin, bool interrupt_enabled);
        ~RotaryEncoder(void);
        bool switch_pressed;

    private:
        uint8_t last_encoder_position; 
        uint8_t current_encoder_position;
        unsigned long last_switch_press;
        uint8_t CLK_PIN;
        uint8_t DT_PIN;
        uint8_t SW_PIN;
        uint8_t instance_id;
        void instanceSwitchISR(void);
        void instanceEncoderISR(void);

    protected:
        static uint8_t isr_instance_flag;
        static RotaryEncoder* encoderISRInstances[];
        static RotaryEncoder* switchISRInstances[];
        static void encoderISR0(void);
        static void switchISR0(void);
        static void encoderISR1(void);
        static void switchISR1(void);
        static void encoderISR2(void);
        static void switchISR2(void);
        static void encoderISR3(void);
        static void switchISR3(void);
        static void encoderISR4(void);
        static void switchISR4(void);
        static void encoderISR5(void);
        static void switchISR5(void);
        static void encoderISR6(void);
        static void switchISR6(void);
        static void encoderISR7(void);
        static void switchISR7(void);
};


#endif	// END ENCODER_H
