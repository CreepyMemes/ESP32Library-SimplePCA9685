#include "PCA9685.h"

/*!
*	@brief Class constructor
*	@param address the device's I2C address which can be 0x70 .. 0x77
*	@param wire	   the I2C Wire object instance
*/
PCA9685::PCA9685(const uint8_t address, TwoWire *wire){
    _address  = address;
    _wire     = wire;
    _lastTime = 0;
    memset(_prevAngles, 0, sizeof(_prevAngles)); // Set all prev angles values to 0
}

/*!
*	@brief  Method that starts the I2C device and initializes some default settings ( tpole mode and 50hz )
*	@return true if successful, otherwise false
*/
bool PCA9685::begin() {
    if (!isConnected()) return false;
    reset();
    return true;
}

/*!
*	@brief Method that sets the PWM frequency of the whole PCA9685 board
*	@param pwmFrequency Min: 24Hz, Max: 1526Hz
*/
void PCA9685::setPWMFrequency(float pwmFrequency) {
    if (pwmFrequency < 0) return;

    // This equation comes from section 7.3.5 of the datasheet, but the rounding has been freq min = 23.84, max = 1525.88.
    int preScalerVal = (25000000 / (4096 * pwmFrequency)) - 1;
    if (preScalerVal > 255) preScalerVal = 255;
    if (preScalerVal < 3)   preScalerVal = 3;

    // The PRE_SCALE register can only be set when the SLEEP bit of MODE1 register is set to logic 1.
    byte mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_PRESCALE_REG, (byte)preScalerVal);

    // It takes 500us max for the oscillator to be up and running once SLEEP bit has been set to logic 0.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    delayMicroseconds(500);
}

/*!
*	@brief Method that sets a PWM value to a selected channel (0 to 15)
*	@param channel The channel to be affected
*	@param pwmAmount The PWM value to be set
*/
void PCA9685::setChannelPWM(int channel, uint16_t pwmAmount) {
    if (channel < 0 || channel > 15) return;

    _wire->beginTransmission(_address);
    _wire->write(PCA9685_LED0_REG + (channel * 0x04));

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(channel, pwmAmount, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);
    _wire->endTransmission();
}

/*!
*	@brief Method that sets an angle to a selected channel (used for servos with pwm: min = 115 / max = 545)
*	@param channel The channel to be affected
*	@param angle   The angle value to be set
*/
void PCA9685::setServoAngle(int channel, int angle) {
    setChannelPWM(channel, map(angle, -90, 90, 115, 545));
}

/*!
*	@brief Method that moves a servo to the selected angle in a smooth manner (must be called every loop)
*	@param channel The channel to which the servo is attached on
*	@param angle   The angle value to move the selected servo to
*	@return returns the current smooth angle position it's setting at
*/
float PCA9685::setSmoothAngle(int channel, float angle){
    uint32_t currentTime = millis();

    if (currentTime - _lastTime >= 10){
        _lastTime = currentTime;

        float smoothAngle = (angle * 0.1) + (_prevAngles[channel] * 0.9);
        _prevAngles[channel] = smoothAngle;

        setServoAngle(channel, smoothAngle);  
    }
    
    return _prevAngles[channel];
}

/*!
*	@brief This method returns the current PWM amount to the selected channel
*	@param channel The channel to be affected
*	@return PWM value is: 0 -> 4096 (0 full off / 4096 full on)
*/
uint16_t PCA9685::getChannelPWM(int channel) {
    if (channel < 0 || channel > 15) return 0;

    byte regAddress = PCA9685_LED0_REG + (channel << 2);

    _wire->beginTransmission(_address);
    _wire->write(regAddress);
    _wire->endTransmission();

    int bytesRead = _wire->requestFrom(_address, (size_t)4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0) lowByte(_wire->read());
        return 0;
    }

    uint16_t phaseBegin =  (uint16_t) lowByte(_wire->read());
             phaseBegin |= (uint16_t) lowByte(_wire->read()) << 8;

    uint16_t phaseEnd    = (uint16_t) lowByte(_wire->read());
             phaseEnd   |= (uint16_t) lowByte(_wire->read()) << 8;

    uint16_t retVal;
    if      (phaseEnd >= PCA9685_PWM_FULL)   retVal = 0;                                          // Full OFF (Figure 11 Example 4)
    else if (phaseBegin >= PCA9685_PWM_FULL) retVal = PCA9685_PWM_FULL;                           // Full ON (Figure 9 Example 3)
    else if (phaseBegin <= phaseEnd)         retVal = phaseEnd - phaseBegin;                      // start and finish in same cycle (Section 7.3.3 example 1)
    else                                     retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin; // span cycles (Section 7.3.3 example 2)

    return retVal;
}

// ---------------------------------------------------- P R I V A T E S --------------------------------------------------------------

/*!
*	@brief  Checks if the device is connected
*	@return Returns false if it fails
*/
bool PCA9685::isConnected(){
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}

/*!
*	@brief  Initializes the device device and sets some default settings
*/
void PCA9685::reset() {
    _wire->beginTransmission(_address);
    _wire->write(PCA9685_SW_RESET);
    _wire->endTransmission();

    writeRegister(PCA9685_MODE1_REG, PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    writeRegister(PCA9685_MODE2_REG, PCA9685_MODE2_OUTDRV_TPOLE);

    setPWMFrequency(50);
}

/*!
*	@brief sets the PWM phases passed by reference
*	@param channel    The channel to be affected
*	@param pwmAmount  The PWM value to be set
*	@param phaseBegin The pointer that will store the output phase begin value
*	@param phaseEnd   The pointer that will store the output phase end value
*/
void PCA9685::getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd){  
    *phaseBegin = 0;
    
    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        *phaseEnd = PCA9685_PWM_FULL;     // Full OFF -> time_end[bit12] = 1
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        *phaseBegin |= PCA9685_PWM_FULL;  // Full ON  -> time_beg[bit12] = 1, time_end[bit12] = <ignored>
        *phaseEnd = 0;
    }
    else {
        *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
    }
}

/*!
*	@brief Sends the PWM value to the I2C device
*	@param phaseBegin The begin phase of the selected PWM
*	@param phaseEnd   The end phase of the selected PWM
*/
void PCA9685::writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
    _wire->write(lowByte(phaseBegin));
    _wire->write(highByte(phaseBegin));
    _wire->write(lowByte(phaseEnd));
    _wire->write(highByte(phaseEnd));
}

/*!
*	@brief Writes some data to a register given it's adress
*	@param regAddress The register's address
*	@param value      The data value that will get written to it
*/
void PCA9685::writeRegister(byte regAddress, byte value) {
    _wire->beginTransmission(_address);
    _wire->write(regAddress);
    _wire->write(value);
    _wire->endTransmission();
}

/*!
*	@brief Reads data from a register given it's adress
*	@param regAddress The register's address
*	@return Returns the data value read from the register
*/
byte PCA9685::readRegister(byte regAddress) {
    _wire->beginTransmission(_address);
    _wire->write(regAddress);
    _wire->endTransmission();

    int bytesRead = _wire->requestFrom(_address, (size_t)1);
    if (bytesRead != 1) {
        while (bytesRead-- > 0) lowByte(_wire->read());
        return 0;
    }

    byte retVal = lowByte(_wire->read());
    return retVal;
}