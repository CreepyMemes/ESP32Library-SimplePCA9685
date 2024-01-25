#pragma once

#include <Arduino.h>
#include <Wire.h>

#define PCA9685_MODE1_REG          0x00
#define PCA9685_MODE2_REG          0x01
#define PCA9685_SUBADR1_REG        0x02
#define PCA9685_SUBADR2_REG        0x03
#define PCA9685_SUBADR3_REG        0x04
#define PCA9685_ALLCALL_REG        0x05
#define PCA9685_LED0_REG           0x06
#define PCA9685_PRESCALE_REG       0xFE
#define PCA9685_ALLLED_REG         0xFA
#define PCA9685_MODE1_RESTART      0x80
#define PCA9685_MODE1_AUTOINC      0x20
#define PCA9685_MODE1_SLEEP        0x10
#define PCA9685_MODE2_OUTDRV_TPOLE 0x04
#define PCA9685_SW_RESET           0x06
#define PCA9685_PWM_FULL           (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK           (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

/*!
	@brief  Class driver for the PCA9685 PWM driver with 16 channels
*/
class PCA9685 {

    public:
        PCA9685(const uint8_t address, TwoWire *wire = &Wire);

        bool     begin();

        void     setPWMFrequency(float pwmFrequency);
        void     setChannelPWM(int channel, uint16_t pwmAmount);
        void     setServoAngle(int channel, int angle);
        float    setSmoothAngle(int channel, float angle);

        uint16_t getChannelPWM(int channel); 

    private:
        bool 	 isConnected(); 
        void     reset();

        void     getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);
        void     writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
        void     writeRegister(byte regAddress, byte value);
        byte     readRegister(byte regAddress);

        float    _prevAngles[16]; // Array that stores the previous angle position of each channel
        uint8_t  _address;        // The I2C device's address
        TwoWire* _wire;           // Pointer to the I2C bus interface
};