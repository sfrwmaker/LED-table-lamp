#ifndef __ENCODER_H
#define __ENCODER_H
#include "main.h"


typedef struct _class_RENC RENC;

void 	RENC_createButton(RENC* renc, GPIO_TypeDef* ButtonPORT, uint16_t ButtonPIN);
void	RENC_createEncoder(RENC* renc, GPIO_TypeDef* aPORT, uint16_t aPIN, GPIO_TypeDef* bPORT, uint16_t bPIN);
void    RENC_resetEncoder(RENC* renc, int16_t initPos, int16_t low, int16_t upp, uint8_t inc, uint8_t fast_inc, bool looped);
void 	RENC_setTimeout(RENC* renc, uint16_t timeout_ms);
void    RENC_setIncrement(RENC* renc, uint8_t inc);
uint8_t	RENC_getIncrement(RENC* renc);
int16_t RENC_read(RENC* renc);
uint8_t	RENC_buttonStatus(RENC* renc);				// The Encoder button current status: 0	- not pressed, 1 - short press, 2 - long press
bool    RENC_write(RENC* renc, int16_t initPos);
void    RENC_encoderIntr(RENC* renc);
int32_t RENC_empAverage(RENC* renc, int32_t value);


struct _class_RENC {
    int16_t          	min_pos, max_pos;
    volatile uint32_t	button_data;                // Exponential average value of button port (to debounce)
    uint16_t          	over_press;                 // Maximum time the button can be pressed (ms)
    volatile uint32_t 	bpt;                        // Time when the button was pressed (press time, ms)
    volatile uint32_t 	rpt;                       	// Time when the encoder was rotated (ms)
    volatile uint32_t 	changed;                  	// Time when the value was changed (ms)
    volatile int16_t  	pos;                       	// Encoder current position
    volatile bool       s_up;						// The status of the secondary channel
    GPIO_TypeDef* 		b_port;						// The PORT of the press button
    GPIO_TypeDef*     	m_port;						// The PORT of the main channel
	GPIO_TypeDef*		s_port;               		// The PORT of the secondary channel
    uint16_t			b_pin, m_pin,  s_pin;	    // The PIN number of the button, main channel and secondary channel
    bool              	is_looped;                  // Whether the encoder is looped
    uint8_t            	increment;                  // The value to add or substract for each encoder tick
    uint8_t             fast_increment;             // The value to change encoder when it runs quickly
    bool				i_b_rel;					// Ignore button release event
    bool				b_on;						// The button current position: true - pressed
    uint32_t			b_check;					// Time when the button should be checked (ms)
};

#endif
