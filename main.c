#include "ch32fun.h"
#include "ch32x00xhw.h"
#include "u8g2.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "u8g2_ch32_hw_i2c.h"

// io
#define ENC_PORT GPIOD
#define ENC_CLK_PIN PD4
#define ENC_DT_PIN PD5
#define ENC_SW_PIN PD6
#define ENC_CLK_PIN_NUM 4
#define ENC_DT_PIN_NUM 5
#define ENC_SW_PIN_NUM 6

#define PIN_ADC0 PA2
#define PIN_ADC1 PA1

#define PWM_HTR0_PIN PD2
#define PWM_HTR1_PIN PD3

// system parms
#define ADC_MAX 4095
#define PWM_PERIOD 999

// timing intervals
#define PID_UPDATE_INTERVAL_MS 20
#define PID_SAMPLE_TIME_S (PID_UPDATE_INTERVAL_MS / 1000.0f)
#define DISPLAY_UPDATE_INTERVAL_MS 100
#define LONG_PRESS_MS 750
#define SHORT_PRESS_DEBOUNCE_MS 50

// pid gains
#define PID_KP_FLOAT 20.0f
#define PID_KI_FLOAT 2.8f
#define PID_KD_FLOAT 4.0f
#define PID_SHIFT 8
#define PID_LOAD_COMP_GAIN 3

// temp and adc
#define ENCODER_TEMP_STEP 10
#define TEMP_CONVERSION_SHIFT 8
#define ADC_FILTER_SHIFT 5
#define MIN_SETTABLE_TEMP_C 50
// below set-point
#define HEATUP_BOOST_THRESHOLD_C 50

// error thresholds
#define ADC_SHORT_CIRCUIT_THRESHOLD 50
#define ADC_DISCONNECTED_THRESHOLD 4080

// sysclock
#define SYSTICK_ONE_MILLISECOND ((uint32_t)FUNCONF_SYSTEM_CORE_CLOCK / 1000)

// ADC ch enum
typedef enum { CHAN_0 = 0, CHAN_1 = 1, NUM_CHANNELS } channel_id_t;

typedef struct {
  uint16_t temp_c;
  uint16_t adc_val;
} cal_point_t;

const cal_point_t cal_points[] = {
    {23, 447},  // room temperature
    {200, 770}, // mid-temperature
    {310, 974},
    {450, 1180} // high-temperature
};
static const int num_cal_points = sizeof(cal_points) / sizeof(cal_point_t);

// max cal point
static const uint16_t MAX_SETTABLE_TEMP_C =
    cal_points[num_cal_points - 1].temp_c;

typedef struct {
  int32_t Kp, Ki, Kd;
  int32_t integral, previous_error;
  int32_t previous_measurement;
  uint16_t min_output, max_output;
  int32_t integral_min, integral_max;
} pid_controller_t;

typedef struct {
  // state
  bool is_on;
  bool has_error;

  // temp & adc
  uint16_t set_point_temp_c;
  uint16_t set_point_adc;
  uint16_t current_adc;
  uint32_t filtered_adc_fixed; // for IIR filter

  // ctl
  pid_controller_t pid;
  uint16_t current_pwm;
} channel_state_t;

volatile uint32_t systick_millis = 0;
volatile int32_t encoder_count = 0;
volatile uint16_t adc_dma_buffer[NUM_CHANNELS] = {0};

u8g2_t u8g2;
channel_state_t channels[NUM_CHANNELS];
uint8_t active_channel = CHAN_0;
int32_t last_encoder_detent_count = 0;

void systick_init(void);
void adc_init(void);
void timers_pwm_init(void);
void encoder_interrupt_init(void);
void display_init(void);
void pid_init(pid_controller_t *pid, uint16_t min_output, uint16_t max_output);
void channels_init(void);
void system_init(void);
void peripherals_init(void);

void handle_inputs(uint32_t current_time);
void update_adc_filters(void);
void update_pid_controllers(uint32_t current_time);
void update_display(uint32_t current_time);
void update_channel_measurements(void);

inline uint32_t millis() { return systick_millis; }
void pwm_set_duty(uint8_t channel, uint16_t duty);
int16_t adc_to_temp_c(uint16_t adc_value);
uint16_t temp_c_to_adc(uint16_t temp_c);
uint16_t pid_update(pid_controller_t *pid, uint16_t set_point,
                    uint16_t current_value);
static inline int32_t constrain(int32_t val, int32_t min, int32_t max);

int main() {
  system_init();
  peripherals_init();
  display_init();
  channels_init();

  while (1) {
    uint32_t current_time = millis();

    handle_inputs(current_time);
    update_pid_controllers(current_time);
    update_display(current_time);
  }
}

void handle_inputs(uint32_t current_time) {
  static uint32_t sw_down_time = 0;
  static bool sw_long_press_handled = false;

  // rot encoder turn
  NVIC_DisableIRQ(EXTI7_0_IRQn);
  int32_t current_encoder_count = encoder_count;
  NVIC_EnableIRQ(EXTI7_0_IRQn);

  int32_t current_encoder_div4 = current_encoder_count / 4;
  int32_t delta = current_encoder_div4 - last_encoder_detent_count;

  if (delta != 0) {
    last_encoder_detent_count = current_encoder_div4;
    int32_t new_temp = (int32_t)channels[active_channel].set_point_temp_c +
                       (delta * ENCODER_TEMP_STEP);

    if (new_temp < MIN_SETTABLE_TEMP_C)
      new_temp = MIN_SETTABLE_TEMP_C;
    if (new_temp > MAX_SETTABLE_TEMP_C)
      new_temp = MAX_SETTABLE_TEMP_C;

    channels[active_channel].set_point_temp_c = (uint16_t)new_temp;
    channels[active_channel].set_point_adc = temp_c_to_adc((uint16_t)new_temp);
    channels[active_channel].pid.integral = 0;
  }

  // encoder sw press
  bool sw_is_down = (funDigitalRead(ENC_SW_PIN) == 0);
  if (sw_is_down) {
    if (sw_down_time == 0) {
      sw_down_time = current_time;
      sw_long_press_handled = false;
    } else if (!sw_long_press_handled &&
               (current_time - sw_down_time >= LONG_PRESS_MS)) {
      active_channel = (active_channel + 1) % NUM_CHANNELS;
      sw_long_press_handled = true;
    }
  } else {
    if (sw_down_time != 0) {
      if (!sw_long_press_handled &&
          (current_time - sw_down_time >= SHORT_PRESS_DEBOUNCE_MS)) {
        channels[active_channel].is_on = !channels[active_channel].is_on;
      }
      sw_down_time = 0;
    }
  }
}

void update_channel_measurements() {
  // turn off heaters, otherwise insane ringing on adc..
  pwm_set_duty(0, 0);
  pwm_set_duty(1, 0);
  // Delay_Ms(1); // delay might be needed for noise to settle

  // trigger adc via dma
  DMA1_Channel1->CFGR &= ~DMA_CFGR1_EN;
  DMA1->INTFCR = DMA1_IT_TC1;
  DMA1_Channel1->CNTR = NUM_CHANNELS;
  DMA1_Channel1->CFGR |= DMA_CFGR1_EN;
  ADC1->CTLR2 |= ADC_SWSTART; // start conversion

  while (!(DMA1->INTFR & DMA1_IT_TC1))
    ;

  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_state_t *ch = &channels[i];
    uint16_t raw_adc = adc_dma_buffer[i];
    bool previous_error_state = ch->has_error;

    // check for sensor errors
    if (raw_adc >= ADC_DISCONNECTED_THRESHOLD ||
        raw_adc <= ADC_SHORT_CIRCUIT_THRESHOLD) {
      ch->has_error = true;
      ch->is_on = false; // turn off on error
    } else {
      ch->has_error = false;
    }

    // IIR low-pass filter to the ADC reading or reset on error
    // maybe not necessary anymore after disabling sampling during pwm
    if (ch->has_error) {
      ch->current_adc = raw_adc;
      ch->filtered_adc_fixed = 0;
    } else {
      uint32_t raw_adc_scaled = (uint32_t)raw_adc << ADC_FILTER_SHIFT;
      if (previous_error_state || ch->filtered_adc_fixed == 0) {
        ch->filtered_adc_fixed = raw_adc_scaled; // init filter
      } else {
        // IIR filter: filtered += (new - filtered) * alpha
        // alpha = 1 / (2^ADC_FILTER_SHIFT)
        ch->filtered_adc_fixed +=
            (((int32_t)raw_adc_scaled - (int32_t)ch->filtered_adc_fixed) >>
             ADC_FILTER_SHIFT);
      }
      ch->current_adc = (uint16_t)(ch->filtered_adc_fixed >> ADC_FILTER_SHIFT);
    }
  }
}

void update_pid_controllers(uint32_t current_time) {
  static uint32_t last_pid_update_time = 0;
  if (current_time - last_pid_update_time < PID_UPDATE_INTERVAL_MS) {
    return;
  }
  last_pid_update_time = current_time;

  update_channel_measurements();

  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_state_t *ch = &channels[i];
    uint16_t target_pwm = 0;

    if (!ch->is_on || ch->has_error) {
      // off or error state
      target_pwm = 0;
      ch->pid.integral = 0;
      ch->pid.previous_measurement = ch->current_adc;

    } else {
      // on and no error state
      int16_t current_temp_c = adc_to_temp_c(ch->current_adc);
      int32_t temp_error_c =
          (int32_t)ch->set_point_temp_c - (int32_t)current_temp_c;

      // boost mode
      if (temp_error_c > HEATUP_BOOST_THRESHOLD_C) {
        target_pwm = ch->pid.max_output;
        // prepare for a bumpless transfer to pid control later.
        ch->pid.integral = 0;
        ch->pid.previous_measurement = ch->current_adc;

      } else {
        // pid mode
        target_pwm = pid_update(&ch->pid, ch->set_point_adc, ch->current_adc);
      }
    }

    ch->current_pwm = target_pwm;
  }

  pwm_set_duty(CHAN_0, channels[CHAN_0].current_pwm);
  pwm_set_duty(CHAN_1, channels[CHAN_1].current_pwm);
}

void update_display(uint32_t current_time) {
  static uint32_t last_display_update_time = 0;
  if (current_time - last_display_update_time < DISPLAY_UPDATE_INTERVAL_MS) {
    return;
  }
  last_display_update_time = current_time;

  char line_buf[40];
  char temp_str[NUM_CHANNELS][10];

  // prep temp strings for both channels
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channels[i].has_error) {
      sprintf(temp_str[i], "%s",
              (channels[i].current_adc <= ADC_SHORT_CIRCUIT_THRESHOLD) ? "SHORT"
                                                                       : "NC");
    } else {
      sprintf(temp_str[i], "%3dC", adc_to_temp_c(channels[i].current_adc));
    }
  }

  u8g2_FirstPage(&u8g2);
  do {
    u8g2_SetFont(&u8g2, u8g2_font_ncenB08_tr);

    // ch0 info
    sprintf(line_buf, "%cCH0 Set: %3uC [%s]", active_channel == 0 ? '>' : ' ',
            channels[CHAN_0].set_point_temp_c,
            channels[0].is_on ? "ON" : "OFF");
    u8g2_DrawStr(&u8g2, 0, 12, line_buf);

    // ch1 info
    sprintf(line_buf, "%cCH1 Set: %3uC [%s]", active_channel == 1 ? '>' : ' ',
            channels[CHAN_1].set_point_temp_c,
            channels[1].is_on ? "ON" : "OFF");
    u8g2_DrawStr(&u8g2, 0, 26, line_buf);

    u8g2_DrawHLine(&u8g2, 0, 32, 128);

    // ch0 measurement
    sprintf(line_buf, "CH0: %5s (%4u)", temp_str[0],
            channels[CHAN_0].current_adc);
    u8g2_DrawStr(&u8g2, 0, 46, line_buf);

    // ch1 measurement
    sprintf(line_buf, "CH1: %5s (%4u)", temp_str[1],
            channels[CHAN_1].current_adc);
    u8g2_DrawStr(&u8g2, 0, 60, line_buf);

  } while (u8g2_NextPage(&u8g2));
}

// init
void system_init(void) {
  SystemInit();
  systick_init();
  funGpioInitAll();
}

void peripherals_init(void) {
  adc_init();
  encoder_interrupt_init();
  timers_pwm_init();
}

void channels_init(void) {
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i] = (channel_state_t){.is_on = false,
                                    .has_error = true,
                                    .set_point_temp_c = 250,
                                    .set_point_adc = 0,
                                    .current_adc = 0,
                                    .filtered_adc_fixed = 0,
                                    .current_pwm = 0};
    channels[i].set_point_adc = temp_c_to_adc(channels[i].set_point_temp_c);
    pid_init(&channels[i].pid, 0, PWM_PERIOD);
  }
}

void systick_init(void) {
  SysTick->CTLR = 0;
  SysTick->CMP = SysTick->CNT + SYSTICK_ONE_MILLISECOND;
  systick_millis = 0;
  SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE | SYSTICK_CTLR_STCLK;
  NVIC_EnableIRQ(SysTicK_IRQn);
}

void adc_init(void) {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1;
  RCC->AHBPCENR |= RCC_AHBPeriph_DMA1;

  funPinMode(PIN_ADC0, GPIO_CFGLR_IN_ANALOG); // PA2 -> ADC_IN0
  funPinMode(PIN_ADC1, GPIO_CFGLR_IN_ANALOG); // PA1 -> ADC_IN1

  RCC->APB2PRSTR |= RCC_APB2Periph_ADC1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_ADC1;

  ADC1->SAMPTR2 = (7 << (3 * CHAN_0)) | (7 << (3 * CHAN_1));
  ADC1->RSQR1 = ((NUM_CHANNELS - 1) << 20);
  ADC1->RSQR3 = (CHAN_0 << 0) | (CHAN_1 << 5);

  ADC1->CTLR1 |= ADC_SCAN; // en scan mode

  ADC1->CTLR2 &= ~ADC_EXTSEL;
  ADC1->CTLR2 |= ADC_EXTSEL_0 | ADC_EXTSEL_1 | ADC_EXTSEL_2; // 111b for SWSTART
  ADC1->CTLR2 |= ADC_EXTTRIG;

  // cal
  ADC1->CTLR2 |= ADC_ADON;
  Delay_Us(1);
  ADC1->CTLR2 |= CTLR2_RSTCAL_Set;
  while (ADC1->CTLR2 & CTLR2_RSTCAL_Set)
    ;
  ADC1->CTLR2 |= CTLR2_CAL_Set;
  while (ADC1->CTLR2 & CTLR2_CAL_Set)
    ;

  DMA1_Channel1->PADDR = (uint32_t)&ADC1->RDATAR;
  DMA1_Channel1->MADDR = (uint32_t)adc_dma_buffer;
  DMA1_Channel1->CNTR = NUM_CHANNELS;
  DMA1_Channel1->CFGR = DMA_M2M_Disable | DMA_Priority_High |
                        DMA_MemoryDataSize_HalfWord |
                        DMA_PeripheralDataSize_HalfWord | DMA_MemoryInc_Enable |
                        DMA_DIR_PeripheralSRC;
  DMA1_Channel1->CFGR |= DMA_CFGR1_EN;

  // ADC1->CTLR2 |= ADC_CONT | ADC_DMA;
  // trigger

  ADC1->CTLR2 |= ADC_DMA;
  // ADC1->CTLR2 |= ADC_ADON;
  // ADC1->CTLR2 |= ADC_SWSTART;
}

void timers_pwm_init(void) {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_TIM1;
  RCC->APB1PCENR |= RCC_APB1Periph_TIM2;

  funPinMode(PWM_HTR0_PIN, GPIO_CFGLR_OUT_10Mhz_AF_PP);
  funPinMode(PWM_HTR1_PIN, GPIO_CFGLR_OUT_10Mhz_AF_PP);

  const uint16_t psc = 1;

  // TIM1 for heater 0 (PD2)
  RCC->APB2PRSTR |= RCC_APB2Periph_TIM1;
  RCC->APB2PRSTR &= ~RCC_APB2Periph_TIM1;
  TIM1->PSC = psc;
  TIM1->ATRLR = PWM_PERIOD;
  TIM1->CHCTLR1 |= TIM1_CHCTLR1_OC1M_2 | TIM1_CHCTLR1_OC1M_1;
  TIM1->CH1CVR = 0;
  TIM1->CCER |= TIM1_CCER_CC1E;
  TIM1->CCER &= ~(TIM1_CCER_CC1P);
  TIM1->BDTR |= TIM1_BDTR_MOE;
  TIM1->SWEVGR |= TIM1_SWEVGR_UG;
  TIM1->CTLR1 |= TIM1_CTLR1_CEN;

  // TIM2 for heater 1 (PD3)
  TIM2->PSC = psc;
  TIM2->ATRLR = PWM_PERIOD;
  TIM2->CHCTLR1 |= TIM2_CHCTLR1_OC2M_2 | TIM2_CHCTLR1_OC2M_1;
  TIM2->CH2CVR = 0;
  TIM2->CCER |= TIM2_CCER_CC2E;
  TIM2->CCER &= ~(TIM2_CCER_CC2P);
  TIM2->SWEVGR |= TIM2_SWEVGR_UG;
  TIM2->CTLR1 |= TIM2_CTLR1_CEN;
}

void encoder_interrupt_init(void) {
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;

  funPinMode(ENC_CLK_PIN, GPIO_CFGLR_IN_PUPD);
  funDigitalWrite(ENC_CLK_PIN, 1);
  funPinMode(ENC_DT_PIN, GPIO_CFGLR_IN_PUPD);
  funDigitalWrite(ENC_DT_PIN, 1);
  funPinMode(ENC_SW_PIN, GPIO_CFGLR_IN_PUPD);
  funDigitalWrite(ENC_SW_PIN, 1);

  AFIO->EXTICR |=
      AFIO_EXTICR_EXTI4_PD | AFIO_EXTICR_EXTI5_PD | AFIO_EXTICR_EXTI6_PD;

  EXTI->RTENR |= EXTI_RTENR_TR4 | EXTI_RTENR_TR5;
  EXTI->FTENR |= EXTI_FTENR_TR4 | EXTI_FTENR_TR5;
  EXTI->INTENR |= EXTI_INTENR_MR4 | EXTI_INTENR_MR5;
  EXTI->INTFR = (EXTI_Line4 | EXTI_Line5);

  NVIC_EnableIRQ(EXTI7_0_IRQn);
}

static inline int32_t constrain(int32_t val, int32_t min, int32_t max) {
  if (val < min)
    return min;
  if (val > max)
    return max;
  return val;
}

void display_init(void) {
  u8g2_Setup_ssd1306_i2c_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_ch32_hw_i2c,
                                         u8x8_dummy_cb);
  u8g2_SetI2CAddress(&u8g2, 0x3C << 1);
  u8g2_InitDisplay(&u8g2);
  u8g2_SetPowerSave(&u8g2, 0);
  u8g2_ClearBuffer(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);
  u8g2_DrawStr(&u8g2, 10, 36, "System Ready");
  u8g2_SendBuffer(&u8g2);
  Delay_Ms(500);
}

void pid_init(pid_controller_t *pid, uint16_t min_output, uint16_t max_output) {
  pid->Kp = (int32_t)(PID_KP_FLOAT * (1 << PID_SHIFT));
  pid->Ki = (int32_t)(PID_KI_FLOAT * PID_SAMPLE_TIME_S * (1 << PID_SHIFT));
  pid->Kd = (int32_t)(PID_KD_FLOAT / PID_SAMPLE_TIME_S * (1 << PID_SHIFT));

  pid->integral = 0;
  pid->previous_error = 0;
  pid->min_output = min_output;
  pid->max_output = max_output;

  // calculate anti-windup limits
  if (pid->Ki > 0) {
    pid->integral_max = (int32_t)(max_output << PID_SHIFT) / pid->Ki;
    pid->integral_min = (int32_t)(min_output << PID_SHIFT) / pid->Ki;
  } else {
    pid->integral_max = 0;
    pid->integral_min = 0;
  }
}

void pwm_set_duty(uint8_t channel, uint16_t duty) {
  if (duty > PWM_PERIOD)
    duty = PWM_PERIOD;
  if (channel == CHAN_0)
    TIM1->CH1CVR = duty;
  else if (channel == CHAN_1)
    TIM2->CH2CVR = duty;
}

uint16_t pid_update(pid_controller_t *pid, uint16_t set_point,
                    uint16_t current_value) {
  // calculate error (proportional on error)
  int32_t error = (int32_t)set_point - (int32_t)current_value;
  int32_t p_term = pid->Kp * error;

  // integral term with anti-windup clamping
  pid->integral += error;
  pid->integral =
      constrain(pid->integral, pid->integral_min, pid->integral_max);
  int32_t i_term = pid->Ki * pid->integral;

  // derivative term (on measurement to prevent derivative kick)
  int32_t d_input = (int32_t)current_value - pid->previous_measurement;
  int32_t d_term = pid->Kd * d_input;
  pid->previous_measurement = (int32_t)current_value;

  // combine terms, scale, and clamp to get the final output
  int32_t output = (p_term + i_term - d_term) >> PID_SHIFT;
  output = constrain(output, pid->min_output, pid->max_output);

  return (uint16_t)output;
}

int16_t adc_to_temp_c(uint16_t adc_value) {
  // find the correct segment in the calibration table for linear interpolation
  for (size_t i = 0; i < num_cal_points - 1; i++) {
    if (adc_value >= cal_points[i].adc_val &&
        adc_value <= cal_points[i + 1].adc_val) {
      int32_t adc_span = cal_points[i + 1].adc_val - cal_points[i].adc_val;
      if (adc_span <= 0)
        return cal_points[i].temp_c;
      int32_t temp_span = cal_points[i + 1].temp_c - cal_points[i].temp_c;
      int32_t adc_delta = adc_value - cal_points[i].adc_val;

      int32_t temp_scaled =
          (cal_points[i].temp_c << TEMP_CONVERSION_SHIFT) +
          (((adc_delta * temp_span) << TEMP_CONVERSION_SHIFT) / adc_span);
      return (int16_t)(temp_scaled >> TEMP_CONVERSION_SHIFT);
    }
  }

  if (adc_value < cal_points[0].adc_val)
    return cal_points[0].temp_c;
  return cal_points[num_cal_points - 1].temp_c;
}

uint16_t temp_c_to_adc(uint16_t temp_c) {
  // find the correct segment in the calibration table for linear interpolation
  for (size_t i = 0; i < num_cal_points - 1; i++) {
    if (temp_c >= cal_points[i].temp_c && temp_c <= cal_points[i + 1].temp_c) {
      int32_t temp_span = cal_points[i + 1].temp_c - cal_points[i].temp_c;
      if (temp_span <= 0)
        return cal_points[i].adc_val;
      int32_t adc_span = cal_points[i + 1].adc_val - cal_points[i].adc_val;
      int32_t temp_delta = temp_c - cal_points[i].temp_c;

      int32_t adc_scaled =
          (cal_points[i].adc_val << TEMP_CONVERSION_SHIFT) +
          (((temp_delta * adc_span) << TEMP_CONVERSION_SHIFT) / temp_span);
      uint16_t final_adc = (uint16_t)(adc_scaled >> TEMP_CONVERSION_SHIFT);
      return constrain(final_adc, 0, ADC_MAX);
    }
  }

  if (temp_c < cal_points[0].temp_c)
    return cal_points[0].adc_val;
  return cal_points[num_cal_points - 1].adc_val;
}

// ISR
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void) {
  SysTick->CMP += SYSTICK_ONE_MILLISECOND;
  SysTick->SR = 0;
  systick_millis++;
}

void EXTI7_0_IRQHandler(void) __attribute__((interrupt));
void EXTI7_0_IRQHandler(void) {
  if (EXTI->INTFR & (EXTI_Line4 | EXTI_Line5)) {
    static volatile uint8_t last_encoder_state = 0;
    static const int8_t q_enc_table[] = {0,  -1, 1, 0, 1, 0, 0,  -1,
                                         -1, 0,  0, 1, 0, 1, -1, 0};

    uint32_t port_state = ENC_PORT->INDR;
    uint8_t current_state = (((port_state >> ENC_CLK_PIN_NUM) & 1) << 1) |
                            ((port_state >> ENC_DT_PIN_NUM) & 1);
    uint8_t index = (last_encoder_state << 2) | current_state;
    encoder_count += q_enc_table[index];
    last_encoder_state = current_state;

    EXTI->INTFR = (EXTI_Line4 | EXTI_Line5);
  }
}
