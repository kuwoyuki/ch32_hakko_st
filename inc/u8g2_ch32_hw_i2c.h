#ifndef _U8G2_CH32_HW_I2C_H
#define _U8G2_CH32_HW_I2C_H

#define U8G2_I2C_CLKRATE 400000
#define U8G2_I2C_PRERATE 2000000
#define U8G2_I2C_TIMEOUT_MAX 250000

#define U8G2_I2C_EVT_MASTER_MODE_SELECT ((uint32_t)0x00030001) // BUSY, MSL, SB
#define U8G2_I2C_EVT_MASTER_TRANSMITTER_SELECTED                               \
  ((uint32_t)0x00070082) // BUSY, MSL, ADDR, TXE, TRA
#define U8G2_I2C_EVT_MASTER_BYTE_TRANSMITTED                                   \
  ((uint32_t)0x00070084) // TRA, BUSY, MSL, TXE, BTF

static inline uint8_t _u8g2_ch32_i2c_chk_evt(uint32_t event_mask) {
  uint32_t status = I2C1->STAR1 | (I2C1->STAR2 << 16);
  return (status & event_mask) == event_mask;
}

static inline void _u8g2_ch32_i2c_hw_init(void) {
  uint16_t tempreg;

  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO;
  RCC->APB1PCENR |= RCC_APB1Periph_I2C1;

  // Mode: 10MHz, Alternate Function, Open-Drain
  // PC1 (SDA)
  GPIOC->CFGLR &= ~(0xf << (4 * 1));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 1);
  // PC2 (SCL)
  GPIOC->CFGLR &= ~(0xf << (4 * 2));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_OD_AF) << (4 * 2);

  RCC->PB1PRSTR |= RCC_APB1Periph_I2C1;
  RCC->PB1PRSTR &= ~RCC_APB1Periph_I2C1;

  tempreg = I2C1->CTLR2;
  tempreg &= ~I2C_CTLR2_FREQ;
  tempreg |= (FUNCONF_SYSTEM_CORE_CLOCK / U8G2_I2C_PRERATE) & I2C_CTLR2_FREQ;
  I2C1->CTLR2 = tempreg;

  // Fast Mode (400kHz)
  tempreg = (FUNCONF_SYSTEM_CORE_CLOCK / (3 * U8G2_I2C_CLKRATE)) &
            I2C_CKCFGR_CCR; // 33% duty
  tempreg |= I2C_CKCFGR_FS;
  I2C1->CKCFGR = tempreg;

  // Enable I2C
  I2C1->CTLR1 |= I2C_CTLR1_PE;
  // set ACK mode
  I2C1->CTLR1 |= I2C_CTLR1_ACK;
}

uint8_t u8x8_byte_ch32_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
                              void *arg_ptr) {
  uint8_t *data;
  int32_t timeout;

  switch (msg) {
  case U8X8_MSG_BYTE_INIT:
    _u8g2_ch32_i2c_hw_init();
    break;

  case U8X8_MSG_BYTE_START_TRANSFER:
    timeout = U8G2_I2C_TIMEOUT_MAX;
    while ((I2C1->STAR2 & I2C_STAR2_BUSY) && (timeout-- > 0))
      ;
    if (timeout <= 0)
      return 0;

    // send START condition
    I2C1->CTLR1 |= I2C_CTLR1_START;

    // wait for master mode select event
    timeout = U8G2_I2C_TIMEOUT_MAX;
    while (!_u8g2_ch32_i2c_chk_evt(U8G2_I2C_EVT_MASTER_MODE_SELECT) &&
           (timeout-- > 0))
      ;
    if (timeout <= 0)
      return 0;

    // send slave address. u8x8->i2c_address is already the 8-bit address (addr
    // << 1)
    I2C1->DATAR = u8x8_GetI2CAddress(u8x8);

    timeout = U8G2_I2C_TIMEOUT_MAX;
    while (!_u8g2_ch32_i2c_chk_evt(U8G2_I2C_EVT_MASTER_TRANSMITTER_SELECTED) &&
           (timeout-- > 0))
      ;
    if (timeout <= 0)
      return 0;
    break;

  case U8X8_MSG_BYTE_SEND:
    data = (uint8_t *)arg_ptr;
    while (arg_int > 0) {
      // wait until transmit buffer is empty (TXE=1)
      timeout = U8G2_I2C_TIMEOUT_MAX;
      while (!(I2C1->STAR1 & I2C_STAR1_TXE) && (timeout-- > 0))
        ;
      if (timeout <= 0)
        return 0;

      I2C1->DATAR = *data++;
      arg_int--;
    }
    break;

  case U8X8_MSG_BYTE_END_TRANSFER:
    // wait until transfer is complete (BTF=1), ensuring the last byte is out
    timeout = U8G2_I2C_TIMEOUT_MAX;
    while (!_u8g2_ch32_i2c_chk_evt(U8G2_I2C_EVT_MASTER_BYTE_TRANSMITTED) &&
           (timeout-- > 0))
      ;
    if (timeout <= 0)
      return 0;

    // send STOP condition
    I2C1->CTLR1 |= I2C_CTLR1_STOP;
    break;

  case U8X8_MSG_BYTE_SET_DC:
    // Not used for I2C
    break;

  default:
    return 0;
  }
  return 1;
}

#endif // _U8G2_CH32_HW_I2C_H