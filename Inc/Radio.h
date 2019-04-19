#ifndef RADIO_H_
#define RADIO_H_

typedef struct {
  uint32_t  capture_rise[8];
  uint32_t  capture_fall[8];
  int16_t  rcADC[8];
  int16_t  rcCommand[7];
} rc;

#define Ch1_PIN         (GPIOA->IDR & GPIO_PIN_0)         /* Timer2 ch1 pin: PA0 */
#define Ch1_POL_RISING  (TIM2->CCER &= ~TIM_CCER_CC1P)    /* Timer2 ch1: rising edge */
#define Ch1_POL_FALLING (TIM2->CCER |= TIM_CCER_CC1P)     /* Timer2 ch1: falling edge */

#define Ch2_PIN 	    (GPIOA->IDR & GPIO_PIN_1)         /* Timer2 ch2 pin: PA1 */
#define Ch2_POL_RISING  (TIM2->CCER &= ~TIM_CCER_CC2P);   /* Timer2 ch2: rising edge */
#define Ch2_POL_FALLING (TIM2->CCER |= TIM_CCER_CC2P);    /* Timer2 ch2: falling edge */

#define Ch3_PIN 	    (GPIOA->IDR & GPIO_PIN_6)         /* Timer3 ch1 pin: PA6 */
#define Ch3_POL_RISING  (TIM3->CCER &= ~TIM_CCER_CC1P);   /* Timer3 ch1: rising edge */
#define Ch3_POL_FALLING (TIM3->CCER |= TIM_CCER_CC1P);    /* Timer3 ch1: falling edge */

#define Ch4_PIN 	    (GPIOA->IDR & GPIO_PIN_7)         /* Timer3 ch2 pin: PA7 */
#define Ch4_POL_RISING  (TIM3->CCER &= ~TIM_CCER_CC2P);   /* Timer3 ch2: rising edge */
#define Ch4_POL_FALLING (TIM3->CCER |= TIM_CCER_CC2P);    /* Timer3 ch2: falling edge */

#define Ch5_PIN 	    (GPIOB->IDR & GPIO_PIN_0)         /* Timer3 ch3 pin: PB0 */
#define Ch5_POL_RISING  (TIM3->CCER &= ~TIM_CCER_CC3P);   /* Timer3 ch3: rising edge */
#define Ch5_POL_FALLING (TIM3->CCER |= TIM_CCER_CC3P);    /* Timer3 ch3: falling edge */

#define Ch6_PIN 	    (GPIOB->IDR & GPIO_PIN_1)         /* Timer3 ch4 pin: PB1 */
#define Ch6_POL_RISING  (TIM3->CCER &= ~TIM_CCER_CC4P);   /* Timer3 ch4: rising edge */
#define Ch6_POL_FALLING (TIM3->CCER |= TIM_CCER_CC4P);    /* Timer3 ch4: falling edge */

//#define Ch7_PIN 	    (GPIOB->IDR & GPIO_PIN_8)         /* Timer4 ch3 pin: PB8 */
//#define Ch7_POL_RISING  (TIM3->CCER &= ~TIM_CCER_CC3P);   /* Timer4 ch3: rising edge */
//#define Ch7_POL_FALLING (TIM3->CCER |= TIM_CCER_CC3P);    /* Timer4 ch3: falling edge */

//#define Ch8_PIN 	    (GPIOB->IDR & GPIO_PIN_9)         /* Timer4 ch4 pin: PB9 */
//#define Ch8_POL_RISING  (TIM4->CCER &= ~TIM_CCER_CC4P);   /* Timer4 ch4: rising edge */
//#define Ch8_POL_FALLING (TIM4->CCER |= TIM_CCER_CC4P);    /* Timer4 ch4: falling edge */

#define ROL_LO (1 << (2 * ROLL))
#define ROL_CE (3 << (2 * ROLL))
#define ROL_HI (2 << (2 * ROLL))
#define PIT_LO (1 << (2 * PITCH))
#define PIT_CE (3 << (2 * PITCH))
#define PIT_HI (2 << (2 * PITCH))
#define YAW_LO (1 << (2 * YAW))
#define YAW_CE (3 << (2 * YAW))
#define YAW_HI (2 << (2 * YAW))
#define THR_LO (1 << (2 * THROTTLE))
#define THR_CE (3 << (2 * THROTTLE))
#define THR_HI (2 << (2 * THROTTLE))

void mwArm(void);
void mwDisarm(void);
void RC_Init(void);
void computeRC(void);

#endif
