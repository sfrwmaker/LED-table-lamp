#include "encoder.h"

#define	RENC_bounce      	50           			/* Bouncing timeout (ms) */
#define	RENC_fast_timeout 	300			        	/* Time in ms to change encoder quickly */
#define RENC_def_over_press	2500					/* Default value for button overpress timeout (ms) */
#define RENC_trigger_on		100						/* avg limit to change button status to on */
#define RENC_trigger_off 	50						/* avg limit to change button status to off */
#define RENC_avg_length   	4						/* avg length */
#define RENC_b_check_period	20						/* The button check period, ms */
#define RENC_long_press		1500					/* If the button was pressed more that this timeout, we assume the long button press */

void 	RENC_createButton(RENC* renc, GPIO_TypeDef* ButtonPORT, uint16_t ButtonPIN) {
	renc->bpt 			= 0;
	renc->b_port 		= ButtonPORT;
	renc->b_pin  		= ButtonPIN;
	renc->over_press	= RENC_def_over_press;
	renc->i_b_rel		= false;
	renc->b_on			= false;
	renc->b_check		= 0;
}

void	RENC_createEncoder(RENC* renc, GPIO_TypeDef* aPORT, uint16_t aPIN, GPIO_TypeDef* bPORT, uint16_t bPIN) {
	renc->rpt		= 0;
	renc->m_port 	= aPORT; renc->s_port = bPORT; renc->m_pin = aPIN; renc->s_pin = bPIN;
	renc->pos 		= 0;
	renc->min_pos 	= -32767; renc->max_pos = 32766; renc->increment = 1;
	renc->changed 	= 0;
	renc->s_up 		= false;
	renc->is_looped = false;
	renc->increment = renc->fast_increment = 1;
}

void    RENC_resetEncoder(RENC* renc, int16_t initPos, int16_t low, int16_t upp, uint8_t inc, uint8_t fast_inc, bool looped) {
	renc->min_pos = low; renc->max_pos = upp;
	if (!RENC_write(renc, initPos)) initPos = renc->min_pos;
	renc->increment = renc->fast_increment = inc;
	if (fast_inc > renc->increment) renc->fast_increment = fast_inc;
	renc->is_looped = looped;
}

void 	RENC_setTimeout(RENC* renc, uint16_t timeout_ms)					{ renc->over_press = timeout_ms; }
void    RENC_setIncrement(RENC* renc, uint8_t inc)            				{ renc->increment = renc->fast_increment = inc; }
uint8_t	RENC_getIncrement(RENC* renc)                 						{ return renc->increment; }
int16_t RENC_read(RENC* renc)                          						{ return renc->pos; }

/*
 * The Encoder button current status
 * 0	- not pressed
 * 1	- short press
 * 2	- long press
 */
uint8_t	RENC_buttonStatus(RENC *renc) {
	if (HAL_GetTick() >= renc->b_check) {			// It is time to check the button status
		renc->b_check = HAL_GetTick() + RENC_b_check_period;
		uint8_t s = 0;
		if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(renc->b_port, renc->b_pin))	// if port state is low, the button pressed
			s = RENC_trigger_on << 1;
		if (renc->b_on) {
			if (RENC_empAverage(renc, s) < RENC_trigger_off)
				renc->b_on = false;
		} else {
			if (RENC_empAverage(renc, s) > RENC_trigger_on)
				renc->b_on = true;
		}

		if (renc->b_on) {                           // Button status is 'pressed'
			uint32_t n = HAL_GetTick() - renc->bpt;
			if ((renc->bpt == 0) || (n > renc->over_press)) {
				renc->bpt = HAL_GetTick();
			} else if (n > RENC_long_press) {     	// Long press
				if (renc->i_b_rel) {
					return 0;
				} else{
					renc->i_b_rel = true;
					return 2;
				}
			}
		} else {                                    // Button status is 'not pressed'
			if (renc->bpt == 0 || renc->i_b_rel) {
				renc->bpt = 0;
				renc->i_b_rel = false;
				return 0;
			}
			uint32_t e = HAL_GetTick() - renc->bpt;
			renc->bpt = 0;							// Ready for next press
			if (e < renc->over_press) {				// Long press already managed
				return 1;
			}
		}
	}
    return 0;
}

bool    RENC_write(RENC* renc, int16_t initPos)	{
	if ((initPos >= renc->min_pos) && (initPos <= renc->max_pos)) {
    renc->pos = initPos;
    return true;
  }
  return false;
}

void RENC_encoderIntr(RENC* renc) {               	// Interrupt function, called when the channel A of encoder changed
  bool mUp = (HAL_GPIO_ReadPin(renc->m_port, renc->m_pin) == GPIO_PIN_SET);
  uint32_t now_t = HAL_GetTick();
  if (!mUp) {                                       // The main channel has been "pressed"
    if ((renc->rpt == 0) || (now_t - renc->rpt > renc->over_press)) {
      renc->rpt = now_t;
      renc->s_up = (HAL_GPIO_ReadPin(renc->s_port, renc->s_pin) == GPIO_PIN_SET);
    }
  } else {
    if (renc->rpt > 0) {
      uint8_t inc = renc->increment;
      if ((now_t - renc->rpt) < renc->over_press) {
        if ((now_t - renc->changed) < RENC_fast_timeout) inc = renc->fast_increment;
        renc->changed = now_t;
        if (renc->s_up) renc->pos -= inc; else renc->pos += inc;
        if (renc->pos > renc->max_pos) {
          if (renc->is_looped)
            renc->pos = renc->min_pos;
          else
            renc->pos = renc->max_pos;
        }
        if (renc->pos < renc->min_pos) {
          if (renc->is_looped)
            renc->pos = renc->max_pos;
          else
            renc->pos = renc->min_pos;
        }
      }
      renc->rpt = 0;
    }
  }
}


int32_t RENC_empAverage(RENC* renc, int32_t value) {
	uint8_t round_v = RENC_avg_length >> 1;
	renc->button_data += value - (renc->button_data + round_v) / RENC_avg_length;
	return (renc->button_data + round_v) / RENC_avg_length;
}
