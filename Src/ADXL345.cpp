/**************************************************************************
 *                                                                         *
 * ADXL345 Driver for Arduino                                              *
 *                                                                         *
 ***************************************************************************
 *                                                                         *
 * This program is free software; you can redistribute it and/or modify    *
 * it under the terms of the GNU License.                                  *
 * This program is distributed in the hope that it will be useful,         *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 * GNU License V2 for more details.                                        *
 *                                                                         *
 ***************************************************************************/

#include "ADXL345.h"
#include <algorithm> // min() & max()
#include <cmath>     // pow()
#include <unistd.h>
#include <fcntl.h>
#include <array>
#include "main.h"


extern void Error_Handler(void);

using namespace std;


#define TO_READ (6)  // num of bytes we are going to read each time (two bytes for each axis)



ADXL345::ADXL345() {
    status      = ADXL345_OK;
    error_code  = ADXL345_NO_ERROR;
    pHi2c       = nullptr;
    dev_address = ADXL345_ADDR_ALT_LOW << 1;

    gains[0] = 0.0039; // 3.9mg/LSB (see datasheet)
    gains[1] = 0.0039;
    gains[2] = 0.0039;
}


bool
ADXL345::init(int16_t _address, I2C_HandleTypeDef *_pHi2c) {
    pHi2c       = _pHi2c;
    dev_address = _address << 1;

    powerOn();
    byte buf;
    readFrom(ADXL345_DEVID, 1, &buf);
    if(buf != ADXL345_IDENTITY)
        Error_Handler();
    setAxisOffset(0, 0, 0);
    return true;
}


void
ADXL345::powerOn() {
    //Turning on the ADXL345
    //writeTo(ADXL345_POWER_CTL, 0);
    //writeTo(ADXL345_POWER_CTL, 16); // AUTOSLEEP
    writeTo(ADXL345_POWER_CTL, 8); // Set the Measure Bit
    HAL_Delay(20);
}


// Reads the acceleration into an array of three places
void
ADXL345::readAccel(int16_t *xyz){
    readAccel(xyz, xyz+1, xyz+2);
}


// Reads the acceleration into three variable x, y and z
void
ADXL345::readAccel(int16_t *x, int16_t *y, int16_t *z) {
    readFrom(ADXL345_DATAX0, TO_READ, buff); //read the acceleration data from the ADXL345

    // each axis reading comes in 10 bit resolution, ie 2 bytes.
    // Least Significat Byte first!!
    // thus we are converting both bytes in to one int
    *x = int16_t((uint16_t(buff[1])) << 8) | buff[0];
    *y = int16_t((uint16_t(buff[3])) << 8) | buff[2];
    *z = int16_t((uint16_t(buff[5])) << 8) | buff[4];
}


void
ADXL345::get_Gxyz(float *xyz){
    int16_t i;
    int16_t xyz_int[3];
    readAccel(xyz_int);
    for(i=0; i<3; i++){
        xyz[i] = float(xyz_int[i]) * gains[i];
    }
}


// Writes val to address register on device
void
ADXL345::writeTo(byte address, byte val) {
    std::array<uint8_t, 2> data{address, val};
    HAL_StatusTypeDef result;
    result = HAL_I2C_Master_Transmit(pHi2c,
                                    (uint16_t)dev_address,
                                    (uint8_t*)data.data(),
                                    data.size(),
                                    10);
    if(result != HAL_OK) {
        Error_Handler();
    }

}


// Reads num bytes starting from address register on device in to _buff array
void
ADXL345::readFrom(byte address, int16_t num, byte _buff[]) {
    HAL_StatusTypeDef result;
    result = HAL_I2C_Master_Transmit(pHi2c,
                                     (uint16_t)dev_address,
                                     (uint8_t*)&address,
                                     1,
                                     10);
    if(result != HAL_OK)
        Error_Handler();
    result = HAL_I2C_Master_Receive(pHi2c,
                                    (uint16_t)dev_address,
                                    (uint8_t *)_buff,
                                    num,
                                    10);
    if(result != HAL_OK)
        Error_Handler();
}


// Gets the range setting and return it into rangeSetting
// it can be 2, 4, 8 or 16
void
ADXL345::getRangeSetting(byte* rangeSetting) {
    byte _b;
    readFrom(ADXL345_DATA_FORMAT, 1, &_b);
    *rangeSetting = _b & 0b00000011;
}


// Sets the range setting, possible values are: 2, 4, 8, 16
void
ADXL345::setRangeSetting(int16_t val) {
    byte _s;
    byte _b;

    switch (val) {
    case 2:
        _s = 0b00000000;
        break;
    case 4:
        _s = 0b00000001;
        break;
    case 8:
        _s = 0b00000010;
        break;
    case 16:
        _s = 0b00000011;
        break;
    default:
        _s = 0b00000000;
    }
    readFrom(ADXL345_DATA_FORMAT, 1, &_b);
    _s |= (_b & 0b11101100);
    writeTo(ADXL345_DATA_FORMAT, _s);
}


// gets the state of the SELF_TEST bit
bool
ADXL345::getSelfTestBit() {
    return getRegisterBit(ADXL345_DATA_FORMAT, 7);
}


// Sets the SELF-TEST bit
// if set to 1 it applies a self-test force to the sensor causing a shift in the output data
// if set to 0 it disables the self-test force
void
ADXL345::setSelfTestBit(bool selfTestBit) {
    setRegisterBit(ADXL345_DATA_FORMAT, 7, selfTestBit);
}


// Gets the state of the SPI bit
bool
ADXL345::getSpiBit() {
    return getRegisterBit(ADXL345_DATA_FORMAT, 6);
}


// Sets the SPI bit
// if set to 1 it sets the device to 3-wire mode
// if set to 0 it sets the device to 4-wire SPI mode
void
ADXL345::setSpiBit(bool spiBit) {
    setRegisterBit(ADXL345_DATA_FORMAT, 6, spiBit);
}


// Gets the state of the INT_INVERT bit
bool
ADXL345::getInterruptLevelBit() {
    return getRegisterBit(ADXL345_DATA_FORMAT, 5);
}


// Sets the INT_INVERT bit
// if set to 0 sets the interrupts to active high
// if set to 1 sets the interrupts to active low
void
ADXL345::setInterruptLevelBit(bool interruptLevelBit) {
    setRegisterBit(ADXL345_DATA_FORMAT, 5, interruptLevelBit);
}


// Gets the state of the FULL_RES bit
bool
ADXL345::getFullResBit() {
    return getRegisterBit(ADXL345_DATA_FORMAT, 3);
}


// Sets the FULL_RES bit
// if set to 1, the device is in full resolution mode, where the output resolution increases with the
//   g range set by the range bits to maintain a 4mg/LSB scal factor
// if set to 0, the device is in 10-bit mode, and the range buts determine the maximum g range
//   and scale factor
void
ADXL345::setFullResBit(bool fullResBit) {
    setRegisterBit(ADXL345_DATA_FORMAT, 3, fullResBit);
}


// Gets the state of the justify bit
bool
ADXL345::getJustifyBit() {
    return getRegisterBit(ADXL345_DATA_FORMAT, 2);
}


// Sets the JUSTIFY bit
// if sets to 1 selects the left justified mode
// if sets to 0 selects right justified mode with sign extension
void
ADXL345::setJustifyBit(bool justifyBit) {
    setRegisterBit(ADXL345_DATA_FORMAT, 2, justifyBit);
}


// Sets the THRESH_TAP byte value
// it should be between 0 and 255
// the scale factor is 62.5 mg/LSB
// A value of 0 may result in undesirable behavior
void
ADXL345::setTapThreshold(int16_t tapThreshold) {
    tapThreshold = min(max(tapThreshold, int16_t(0)), int16_t(255));
    byte _b = byte (tapThreshold);
    writeTo(ADXL345_THRESH_TAP, _b);
}


// Gets the THRESH_TAP byte value
// return value is comprised between 0 and 255
// the scale factor is 62.5 mg/LSB
int16_t
ADXL345::getTapThreshold() {
    byte _b;
    readFrom(ADXL345_THRESH_TAP, 1, &_b);
    return int16_t (_b);
}


// set/get the gain for each axis in Gs / count
void
ADXL345::setAxisGains(float *_gains){
    int16_t i;
    for(i=0; i<3; i++){
        gains[i] = _gains[i];
    }
}


void
ADXL345::getAxisGains(float *_gains){
    int16_t i;
    for(i=0; i<3; i++){
        _gains[i] = gains[i];
    }
}


// Sets the OFSX, OFSY and OFSZ bytes
// OFSX, OFSY and OFSZ are user offset adjustments in twos complement format with
// a scale factor of 15,6mg/LSB
// OFSX, OFSY and OFSZ should be comprised between
void
ADXL345::setAxisOffset(int16_t x, int16_t y, int16_t z) {
    writeTo(ADXL345_OFSX, byte (x));
    writeTo(ADXL345_OFSY, byte (y));
    writeTo(ADXL345_OFSZ, byte (z));
}


// Gets the OFSX, OFSY and OFSZ bytes
void
ADXL345::getAxisOffset(int16_t* x, int16_t* y, int16_t*z) {
    byte _b;
    readFrom(ADXL345_OFSX, 1, &_b);
    *x = int16_t (_b);
    readFrom(ADXL345_OFSY, 1, &_b);
    *y = int16_t (_b);
    readFrom(ADXL345_OFSZ, 1, &_b);
    *z = int16_t (_b);
}


// Sets the DUR byte
// The DUR byte contains an unsigned time value representing the maximum time
// that an event must be above THRESH_TAP threshold to qualify as a tap event
// The scale factor is 625µs/LSB
// A value of 0 disables the tap/float tap funcitons. Max value is 255.
void
ADXL345::setTapDuration(int16_t tapDuration) {
    tapDuration = min(max(tapDuration, int16_t(0)), int16_t(255));
    byte _b = byte (tapDuration);
    writeTo(ADXL345_DUR, _b);
}


// Gets the DUR byte
int16_t ADXL345::getTapDuration() {
    byte _b;
    readFrom(ADXL345_DUR, 1, &_b);
    return int16_t (_b);
}


// Sets the latency (latent register) which contains an unsigned time value
// representing the wait time from the detection of a tap event to the start
// of the time window, during which a possible second tap can be detected.
// The scale factor is 1.25ms/LSB. A value of 0 disables the float tap function.
// It accepts a maximum value of 255.
void
ADXL345::setDoubleTapLatency(int16_t floatTapLatency) {
    byte _b = byte (floatTapLatency);
    writeTo(ADXL345_LATENT, _b);
}


// Gets the Latent value
int16_t
ADXL345::getDoubleTapLatency() {
    byte _b;
    readFrom(ADXL345_LATENT, 1, &_b);
    return int16_t (_b);
}


// Sets the Window register, which contains an unsigned time value representing
// the amount of time after the expiration of the latency time (Latent register)
// during which a second valud tap can begin. The scale factor is 1.25ms/LSB. A
// value of 0 disables the float tap function. The maximum value is 255.
void
ADXL345::setDoubleTapWindow(int16_t floatTapWindow) {
    floatTapWindow = min(max(floatTapWindow, int16_t(0)), int16_t(255));
    byte _b = byte (floatTapWindow);
    writeTo(ADXL345_WINDOW, _b);
}


// Gets the Window register
int16_t
ADXL345::getDoubleTapWindow() {
    byte _b;
    readFrom(ADXL345_WINDOW, 1, &_b);
    return int16_t (_b);
}


// Sets the THRESH_ACT byte which holds the threshold value for detecting activity.
// The data format is unsigned, so the magnitude of the activity event is compared
// with the value is compared with the value in the THRESH_ACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// activity interrupt is enabled. The maximum value is 255.
void
ADXL345::setActivityThreshold(int16_t activityThreshold) {
    activityThreshold = min(max(activityThreshold, int16_t(0)), int16_t(255));
    byte _b = byte (activityThreshold);
    writeTo(ADXL345_THRESH_ACT, _b);
}


// Gets the THRESH_ACT byte
int16_t
ADXL345::getActivityThreshold() {
    byte _b;
    readFrom(ADXL345_THRESH_ACT, 1, &_b);
    return int16_t (_b);
}


// Sets the THRESH_INACT byte which holds the threshold value for detecting inactivity.
// The data format is unsigned, so the magnitude of the inactivity event is compared
// with the value is compared with the value in the THRESH_INACT register. The scale
// factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the
// inactivity interrupt is enabled. The maximum value is 255.
void
ADXL345::setInactivityThreshold(int16_t inactivityThreshold) {
    inactivityThreshold = min(max(inactivityThreshold, int16_t(0)), int16_t(255));
    byte _b = byte (inactivityThreshold);
    writeTo(ADXL345_THRESH_INACT, _b);
}


// Gets the THRESH_INACT byte
int16_t
ADXL345::getInactivityThreshold() {
    byte _b;
    readFrom(ADXL345_THRESH_INACT, 1, &_b);
    return int16_t (_b);
}


// Sets the TIME_INACT register, which contains an unsigned time value representing the
// amount of time that acceleration must be less thant the value in the THRESH_INACT
// register for inactivity to be declared. The scale factor is 1sec/LSB. The value must
// be between 0 and 255.
void
ADXL345::setTimeInactivity(int16_t timeInactivity) {
    timeInactivity = min(max(timeInactivity, int16_t(0)), int16_t(255));
    byte _b = byte (timeInactivity);
    writeTo(ADXL345_TIME_INACT, _b);
}


// Gets the TIME_INACT register
int16_t
ADXL345::getTimeInactivity() {
    byte _b;
    readFrom(ADXL345_TIME_INACT, 1, &_b);
    return int16_t (_b);
}


// Sets the THRESH_FF register which holds the threshold value, in an unsigned format, for
// free-fall detection. The root-sum-square (RSS) value of all axes is calculated and
// compared whith the value in THRESH_FF to determine if a free-fall event occured. The
// scale factor is 62.5mg/LSB. A value of 0 may result in undesirable behavior if the free-fall
// interrupt is enabled. The maximum value is 255.
void
ADXL345::setFreeFallThreshold(int16_t freeFallThreshold) {
    freeFallThreshold = min(max(freeFallThreshold, int16_t(0)), int16_t(255));
    byte _b = byte (freeFallThreshold);
    writeTo(ADXL345_THRESH_FF, _b);
}


// Gets the THRESH_FF register.
int16_t
ADXL345::getFreeFallThreshold() {
    byte _b;
    readFrom(ADXL345_THRESH_FF, 1, &_b);
    return int16_t (_b);
}


// Sets the TIME_FF register, which holds an unsigned time value representing the minimum
// time that the RSS value of all axes must be less than THRESH_FF to generate a free-fall
// interrupt. The scale factor is 5ms/LSB. A value of 0 may result in undesirable behavior if
// the free-fall interrupt is enabled. The maximum value is 255.
void
ADXL345::setFreeFallDuration(int16_t freeFallDuration) {
    freeFallDuration = min(max(freeFallDuration, int16_t(0)), int16_t(255));
    byte _b = byte (freeFallDuration);
    writeTo(ADXL345_TIME_FF, _b);
}


// Gets the TIME_FF register.
int16_t
ADXL345::getFreeFallDuration() {
    byte _b;
    readFrom(ADXL345_TIME_FF, 1, &_b);
    return int16_t (_b);
}


bool
ADXL345::isActivityXEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 6);
}


bool
ADXL345::isActivityYEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 5);
}


bool
ADXL345::isActivityZEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 4);
}


bool
ADXL345::isInactivityXEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 2);
}


bool
ADXL345::isInactivityYEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 1);
}


bool
ADXL345::isInactivityZEnabled() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 0);
}


void
ADXL345::setActivityX(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 6, state);
}


void
ADXL345::setActivityY(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 5, state);
}


void
ADXL345::setActivityZ(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 4, state);
}


void
ADXL345::setInactivityX(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 2, state);
}


void
ADXL345::setInactivityY(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 1, state);
}


void
ADXL345::setInactivityZ(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 0, state);
}


bool
ADXL345::isActivityAc() {
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 7);
}


bool
ADXL345::isInactivityAc(){
    return getRegisterBit(ADXL345_ACT_INACT_CTL, 3);
}


void
ADXL345::setActivityAc(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 7, state);
}


void
ADXL345::setInactivityAc(bool state) {
    setRegisterBit(ADXL345_ACT_INACT_CTL, 3, state);
}


bool
ADXL345::getSuppressBit(){
    return getRegisterBit(ADXL345_TAP_AXES, 3);
}


void
ADXL345::setSuppressBit(bool state) {
    setRegisterBit(ADXL345_TAP_AXES, 3, state);
}


bool
ADXL345::isTapDetectionOnX(){
    return getRegisterBit(ADXL345_TAP_AXES, 2);
}


void
ADXL345::setTapDetectionOnX(bool state) {
    setRegisterBit(ADXL345_TAP_AXES, 2, state);
}


bool
ADXL345::isTapDetectionOnY(){
    return getRegisterBit(ADXL345_TAP_AXES, 1);
}


void
ADXL345::setTapDetectionOnY(bool state) {
    setRegisterBit(ADXL345_TAP_AXES, 1, state);
}


bool
ADXL345::isTapDetectionOnZ(){
    return getRegisterBit(ADXL345_TAP_AXES, 0);
}


void
ADXL345::setTapDetectionOnZ(bool state) {
    setRegisterBit(ADXL345_TAP_AXES, 0, state);
}


bool
ADXL345::isActivitySourceOnX(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 6);
}


bool
ADXL345::isActivitySourceOnY(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 5);
}


bool
ADXL345::isActivitySourceOnZ(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 4);
}


bool
ADXL345::isTapSourceOnX(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 2);
}


bool
ADXL345::isTapSourceOnY(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 1);
}


bool
ADXL345::isTapSourceOnZ(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 0);
}


bool
ADXL345::isAsleep(){
    return getRegisterBit(ADXL345_ACT_TAP_STATUS, 3);
}


bool
ADXL345::isLowPower(){
    return getRegisterBit(ADXL345_BW_RATE, 4);
}


void
ADXL345::setLowPower(bool state) {
    setRegisterBit(ADXL345_BW_RATE, 4, state);
}


float
ADXL345::getRate(){
    byte _b;
    readFrom(ADXL345_BW_RATE, 1, &_b);
    _b &= 0b00001111;
    return (pow(2,((int) _b)-6)) * 6.25;
}


void
ADXL345::setRate(float rate){
    byte _b,_s;
    int16_t v = (int16_t) (rate / 6.25);
    int16_t r = 0;
    while(v >>= 1) {
        r++;
    }
    if (r <= 9) {
        readFrom(ADXL345_BW_RATE, 1, &_b);
        _s = (byte) (r + 6) | (_b & 0b11110000);
        writeTo(ADXL345_BW_RATE, _s);
    }
}


void
ADXL345::set_bw(byte bw_code){
    if((bw_code < ADXL345_BW_3) || (bw_code > ADXL345_BW_1600)) {
        status = false;
        error_code = ADXL345_BAD_ARG;
    }
    else {
        writeTo(ADXL345_BW_RATE, bw_code);
    }
}


byte
ADXL345::get_bw_code(){
    byte bw_code;
    readFrom(ADXL345_BW_RATE, 1, &bw_code);
    return bw_code;
}


byte
ADXL345::getInterruptSource() {
    byte _b;
    readFrom(ADXL345_INT_SOURCE, 1, &_b);
    return _b;
}


bool
ADXL345::getInterruptSource(byte interruptBit) {
    return getRegisterBit(ADXL345_INT_SOURCE,interruptBit);
}


bool
ADXL345::getInterruptMapping(byte interruptBit) {
    return getRegisterBit(ADXL345_INT_MAP,interruptBit);
}


// Set the mapping of an interrupt to pin1 or pin2
// eg: setInterruptMapping(ADXL345_INT_DOUBLE_TAP_BIT,ADXL345_INT2_PIN);
void
ADXL345::setInterruptMapping(byte interruptBit, bool interruptPin) {
    setRegisterBit(ADXL345_INT_MAP, interruptBit, interruptPin);
}


bool
ADXL345::isInterruptEnabled(byte interruptBit) {
    return getRegisterBit(ADXL345_INT_ENABLE, interruptBit);
}


void
ADXL345::setInterrupt(byte interruptBit, bool state) {
    setRegisterBit(ADXL345_INT_ENABLE, interruptBit, state);
}


void
ADXL345::setRegisterBit(byte regAdress, int16_t bitPos, bool state) {
    byte _b;
    readFrom(regAdress, 1, &_b);
    if(state) {
        _b |= (1 << bitPos);  // forces nth bit of _b to be 1.  all other bits left alone.
    }
    else {
        _b &= ~(1 << bitPos); // forces nth bit of _b to be 0.  all other bits left alone.
    }
    writeTo(regAdress, _b);
}


bool
ADXL345::getRegisterBit(byte regAdress, int16_t bitPos) {
    byte _b;
    readFrom(regAdress, 1, &_b);
    return ((_b >> bitPos) & 1);
}


// print all register value to the serial ouptut, which requires it to be setup
// this can be used to manually to check the current configuration of the device
void
ADXL345::printAllRegister() {
//    byte _b;
//    QString sString;
//    readFrom(0x00, 1, &_b);
//    sString = QString("0x00: %1").arg(_b, 0, 2, QLatin1Char('0'));
//    qDebug() << sString;
//    int16_t i;
//    for(i=29; i<=57; i++) {
//        readFrom(i, 1, &_b);
//        sString = QString("0x%1: %2").arg(i, 2, 16, QLatin1Char('0')).arg(_b, 8, 2, QLatin1Char('0'));
//        qDebug() << sString;
//    }
}

