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
#include <sys/types.h>


/*
 *	Defines
 */

#define CLK_BIT                 (0x01)
#define DT_BIT                  (0x00)
#define SW_TIMEOUT_MS           (100)
#define MAX_ENCODER_ISR         (8)
#define DIR_NO_MOVE             (0x00)
#define DIR_CW                  (0x10)
#define DIR_CCW                 (0x20)
#define ENCODER_START_STATE     (0x0)
#define ENCODER_CW_END          (0x1)
#define ENCODER_CW_BEGIN        (0x2)
#define ENCODER_CW_NEXT         (0x3)
#define ENCODER_CCW_BEGIN       (0x4)
#define ENCODER_CCW_END         (0x5)
#define ENCODER_CCW_NEXT        (0x6)


/*
 *  Class
 */

class RotaryEncoder {
    public:
        uint8_t getDirection(void);
        bool switchPressed(void);
        bool isAvailable(void);
        RotaryEncoder(uint8_t rotary_clk_pin, uint8_t rotary_dt_pin, 
            uint8_t rotary_sw_pin, bool interrupt_enabled);
        ~RotaryEncoder(void);
        

    private:
        const uint8_t state_table[7][4] = {
            {ENCODER_START_STATE, ENCODER_CW_BEGIN, ENCODER_CCW_BEGIN, ENCODER_START_STATE},
            {ENCODER_CW_NEXT, ENCODER_START_STATE, ENCODER_CW_END, ENCODER_START_STATE | DIR_CW},
            {ENCODER_CW_NEXT, ENCODER_CW_BEGIN, ENCODER_START_STATE, ENCODER_START_STATE},
            {ENCODER_CW_NEXT, ENCODER_CW_BEGIN, ENCODER_CW_END, ENCODER_START_STATE},
            {ENCODER_CCW_NEXT, ENCODER_START_STATE, ENCODER_CCW_BEGIN, ENCODER_START_STATE},
            {ENCODER_CCW_NEXT, ENCODER_CCW_END, ENCODER_START_STATE, ENCODER_START_STATE | DIR_CCW},
            {ENCODER_CCW_NEXT, ENCODER_CCW_END, ENCODER_CCW_BEGIN, ENCODER_START_STATE}
        }; 
        uint8_t encoder_state;
        uint8_t last_encoder_position; 
        uint8_t current_encoder_position;
        unsigned long last_switch_press;
        uint8_t CLK_PIN;
        uint8_t DT_PIN;
        uint8_t SW_PIN;
        uint8_t instance_id;
        bool switch_pressed;
        void instanceSwitchISR(void);
        void instanceEncoderISR(void);

    protected:
        static uint8_t isr_instance_flag;
        static RotaryEncoder* ISRInstances[];
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
