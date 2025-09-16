#ifndef _I2C_BITBANG_HAL_H
#define _I2C_BITBANG_HAL_H

#ifndef I2C_ADDR_7BIT
#define I2C_ADDR_7BIT 0x3c
#endif

#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN PC1
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN PC2
#endif

#define SCL_HIGH() funDigitalWrite(I2C_SCL_PIN, 1)
#define SCL_LOW() funDigitalWrite(I2C_SCL_PIN, 0)
#define SDA_HIGH() funDigitalWrite(I2C_SDA_PIN, 1)
#define SDA_LOW() funDigitalWrite(I2C_SDA_PIN, 0)

static void i2c_hal_setup(void) {
  funGpioInitAll();
  funPinMode(I2C_SDA_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
  funDigitalWrite(I2C_SDA_PIN, 1);
  funPinMode(I2C_SCL_PIN, GPIO_CFGLR_OUT_10Mhz_PP);
  funDigitalWrite(I2C_SCL_PIN, 1);
}

#endif // _I2C_BITBANG_HAL_H