/****************************************************************************
* ITG3200.cpp - ITG-3200/I2C library v0.5 for Arduino                         *
* Copyright 2010-2011 Filipe Vieira & various contributors                  *
* http://code.google.com/p/itg-3200driver                                   *
* This file is part of ITG-3200 Arduino library.                            *
*                                                                           *
* This library is free software: you can redistribute it and/or modify      *
* it under the terms of the GNU Lesser General Public License as published  *
* by the Free Software Foundation, either version 3 of the License, or      *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU Lesser General Public License for more details.                       *
*                                                                           *
* You should have received a copy of the GNU Lesser General Public License  *
* along with this program.  If not, see <http://www.gnu.org/licenses/>.     *
****************************************************************************/

#include "ITG3200.h"
#include <unistd.h>
#include <fcntl.h>
#include <array>
#include "main.h"

extern void Error_Handler(void);

using namespace std;


ITG3200::ITG3200() {
    pHi2c       = nullptr;
    dev_address = ITG3200_ADDR_AD0_LOW << 1;
    setGains(1.0, 1.0, 1.0);
    setOffsets(0.0, 0.0, 0.0);
    setRevPolarity(false, false, false);
}


bool
ITG3200::init(uint16_t  _address, I2C_HandleTypeDef *_pHi2c) {
    pHi2c       = _pHi2c;
    dev_address = _address << 1;

    reset(); // Reset to Power_On conditions

    readmem(WHO_AM_I, 3, &buff[0]); // Presence Check
    if(buff[0] != ITG3200_ADDR_AD0_LOW)
        Error_Handler();

    // Uncomment or change your default ITG3200 initialization

    // fast sample rate - divisor = 0 filter = 0 clocksrc = 0, 1, 2, or 3  (raw values)
    init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_XGYRO_REF, 1, 1);

    // slow sample rate - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 0, 1, 2, or 3  (raw values)
    //init(NOSRDIVIDER, RANGE2000, BW010_SR1, INTERNALOSC, 1, 1);

    // fast sample rate 32Khz external clock - divisor = 0  filter = 0  clocksrc = 4  (raw values)
    //init(NOSRDIVIDER, RANGE2000, BW256_SR8, PLL_EXTERNAL32, 1, 1);

    // slow sample rate 32Khz external clock - divisor = 0  filter = 1,2,3,4,5, or 6  clocksrc = 4  (raw values)
    //init(NOSRDIVIDER, RANGE2000, BW010_SR1, PLL_EXTERNAL32, 1, 1);
    return true;
}


void
ITG3200::init(byte _SRateDiv, byte _Range, byte _filterBW, byte _ClockSrc, uint8_t _ITGReady, uint8_t _INTRawDataReady) {
    HAL_Delay(GYROSTART_UP_DELAY);
    setClockSource(_ClockSrc);
    HAL_Delay(GYROSTART_UP_DELAY);

    setSampleRateDiv(_SRateDiv);
    HAL_Delay(GYROSTART_UP_DELAY);
    setFSRange(_Range);
    HAL_Delay(GYROSTART_UP_DELAY);
    setFilterBW(_filterBW);
    HAL_Delay(GYROSTART_UP_DELAY);
    setRawDataReady(_INTRawDataReady);
    HAL_Delay(GYROSTART_UP_DELAY);
    setLatchMode(UNTIL_INT_CLEARED);  // Interrupt stay ON until cleared
    HAL_Delay(GYROSTART_UP_DELAY);
    setLatchClearMode(READ_STATUSREG);// Interrupt Cleared by Reading Status Register
    HAL_Delay(GYROSTART_UP_DELAY);
    setITGReady(_ITGReady);
    HAL_Delay(GYROSTART_UP_DELAY);
    while(!isITGReadyOn()) {
    }
}


byte
ITG3200::getDevAddr() {
    return dev_address;
}


void
ITG3200::setDevAddr(uint16_t  _addr) {
    writemem(WHO_AM_I, _addr);
    dev_address = _addr;
}


byte
ITG3200::getSampleRateDiv() {
    readmem(SMPLRT_DIV, 1, &buff[0]);
    return buff[0];
}


void
ITG3200::setSampleRateDiv(byte _SampleRate) {
    writemem(SMPLRT_DIV, _SampleRate);
}


byte
ITG3200::getFSRange() {
    readmem(DLPF_FS, 1, &buff[0]);
    return ((buff[0] & DLPFFS_FS_SEL) >> 3);
}


void
ITG3200::setFSRange(byte _Range) {
    readmem(DLPF_FS, 1, &buff[0]);
    writemem(DLPF_FS, ((buff[0] & ~DLPFFS_FS_SEL) | (_Range << 3)) );
}


byte
ITG3200::getFilterBW() {
    readmem(DLPF_FS, 1, &buff[0]);
    return (buff[0] & DLPFFS_DLPF_CFG);
}


void
ITG3200::setFilterBW(byte _BW) {
    readmem(DLPF_FS, 1, &buff[0]);
    writemem(DLPF_FS, ((buff[0] & ~DLPFFS_DLPF_CFG) | _BW));
}


bool
ITG3200::isINTActiveOnLow() {
    readmem(INT_CFG, 1, &buff[0]);
    return ((buff[0] & INTCFG_ACTL) >> 7);
}


void
ITG3200::setINTLogiclvl(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_ACTL) | (_State << 7)));
}


bool
ITG3200::isINTOpenDrain() {
    readmem(INT_CFG, 1, &buff[0]);
    return ((buff[0] & INTCFG_OPEN) >> 6);
}


void
ITG3200::setINTDriveType(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_OPEN) | _State << 6));
}


bool
ITG3200::isLatchUntilCleared() {
    readmem(INT_CFG, 1, &buff[0]);
    return ((buff[0] & INTCFG_LATCH_INT_EN) >> 5);
}


void
ITG3200::setLatchMode(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_LATCH_INT_EN) | _State << 5));
}


bool
ITG3200::isAnyRegClrMode() {
    readmem(INT_CFG, 1, &buff[0]);
    return ((buff[0] & INTCFG_INT_ANYRD_2CLEAR) >> 4);
}


void
ITG3200::setLatchClearMode(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_INT_ANYRD_2CLEAR) | _State << 4));
}


bool
ITG3200::isITGReadyOn() {
    readmem(INT_CFG, 1, &buff[0]);
    return ((buff[0] & INTCFG_ITG_RDY_EN) >> 2);
}


void
ITG3200::setITGReady(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_ITG_RDY_EN) | _State << 2));
}


bool
ITG3200::isRawDataReadyOn() {
    readmem(INT_CFG, 1, &buff[0]);
    return (buff[0] & INTCFG_RAW_RDY_EN);
}


void
ITG3200::setRawDataReady(uint8_t _State) {
    readmem(INT_CFG, 1, &buff[0]);
    writemem(INT_CFG, ((buff[0] & ~INTCFG_RAW_RDY_EN) | _State));
}


bool
ITG3200::isITGReady() {
    readmem(INT_STATUS, 1, &buff[0]);
    return ((buff[0] & INTSTATUS_ITG_RDY) >> 2);
}


bool
ITG3200::isRawDataReady() {
    isRawDataReadyOn();
    readmem(INT_STATUS, 1, &buff[0]);
    return (buff[0] & INTSTATUS_RAW_DATA_RDY);
}


void
ITG3200::readTemp(float *_Temp) {
    readmem(TEMP_OUT, 2, buff);
    *_Temp = (buff[0] << 8) | buff[1];
}


void
ITG3200::setRevPolarity(bool _Xpol, bool _Ypol, bool _Zpol) {
    polarities[0] = _Xpol ? -1 : 1;
    polarities[1] = _Ypol ? -1 : 1;
    polarities[2] = _Zpol ? -1 : 1;
}


void
ITG3200::setGains(float _Xgain, float _Ygain, float _Zgain) {
    gains[0] = _Xgain;
    gains[1] = _Ygain;
    gains[2] = _Zgain;
}


void
ITG3200::setOffsets(int16_t _Xoffset, int16_t _Yoffset, int16_t _Zoffset) {
    offsets[0] = _Xoffset;
    offsets[1] = _Yoffset;
    offsets[2] = _Zoffset;
}


void
ITG3200::zeroCalibrate(uint16_t totSamples) {
    int16_t xyz[3];
    float tmpOffsets[] = {0, 0, 0};
    while(!isITGReadyOn()) {
    }
    for(uint16_t i=0; i<totSamples; i++) {
        while(!isRawDataReady()) {}
        readGyroRaw(xyz);
        tmpOffsets[0] += float(xyz[0]);
        tmpOffsets[1] += float(xyz[1]);
        tmpOffsets[2] += float(xyz[2]);
    }
    setOffsets(-int16_t(tmpOffsets[0]/float(totSamples)+0.5),
               -int16_t(tmpOffsets[1]/float(totSamples)+0.5),
               -int16_t(tmpOffsets[2]/float(totSamples)+0.5));
}


void
ITG3200::readGyroRaw(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ){
    readmem(GYRO_XOUT, 6, buff);
    *_GyroX = int16_t((uint16_t(buff[0]) << 8) | buff[1]);
    *_GyroY = int16_t((uint16_t(buff[2]) << 8) | buff[3]);
    *_GyroZ = int16_t((uint16_t(buff[4]) << 8) | buff[5]);
}


void
ITG3200::readGyroRaw(int16_t *_GyroXYZ){
    readGyroRaw(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::readGyroRawCal(int16_t *_GyroX, int16_t *_GyroY, int16_t *_GyroZ) {
    readGyroRaw(_GyroX, _GyroY, _GyroZ);
    *_GyroX += offsets[0];
    *_GyroY += offsets[1];
    *_GyroZ += offsets[2];
}


void
ITG3200::readGyroRawCal(int16_t *_GyroXYZ) {
    readGyroRawCal(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::readGyro(float *_GyroX, float *_GyroY, float *_GyroZ){
    int16_t x, y, z;
     // x,y,z will contain calibrated integer values from the sensor
    readGyroRawCal(&x, &y, &z);
    // sensitivity is 14.375 LSBs per °/sec
    *_GyroX =  (float(x) / 14.375) * polarities[0] * gains[0];
    *_GyroY =  (float(y) / 14.375) * polarities[1] * gains[1];
    *_GyroZ =  (float(z) / 14.375) * polarities[2] * gains[2];
}


void
ITG3200::readGyro(float *_GyroXYZ){
    readGyro(_GyroXYZ, _GyroXYZ+1, _GyroXYZ+2);
}


void
ITG3200::reset() {
    writemem(PWR_MGM, PWRMGM_HRESET);
    HAL_Delay(4*GYROSTART_UP_DELAY); //gyro startup
}


bool
ITG3200::isLowPower() {
    readmem(PWR_MGM, 1, &buff[0]);
    return (buff[0] & PWRMGM_SLEEP) >> 6;
}


void
ITG3200::setPowerMode(uint8_t _State) {
    readmem(PWR_MGM, 1, &buff[0]);
    writemem(PWR_MGM, ((buff[0] & ~PWRMGM_SLEEP) | _State << 6));
}


bool
ITG3200::isXgyroStandby() {
    readmem(PWR_MGM, 1, &buff[0]);
    return (buff[0] & PWRMGM_STBY_XG) >> 5;
}


bool
ITG3200::isYgyroStandby() {
    readmem(PWR_MGM, 1, &buff[0]);
    return (buff[0] & PWRMGM_STBY_YG) >> 4;
}


bool
ITG3200::isZgyroStandby() {
    readmem(PWR_MGM, 1, &buff[0]);
    return (buff[0] & PWRMGM_STBY_ZG) >> 3;
}


void
ITG3200::setXgyroStandby(uint8_t _Status) {
    readmem(PWR_MGM, 1, &buff[0]);
    writemem(PWR_MGM, ((buff[0] & PWRMGM_STBY_XG) | _Status << 5));
}


void
ITG3200::setYgyroStandby(uint8_t _Status) {
    readmem(PWR_MGM, 1, &buff[0]);
    writemem(PWR_MGM, ((buff[0] & PWRMGM_STBY_YG) | _Status << 4));
}


void
ITG3200::setZgyroStandby(uint8_t _Status) {
    readmem(PWR_MGM, 1, &buff[0]);
    writemem(PWR_MGM, ((buff[0] & PWRMGM_STBY_ZG) | _Status << 3));
}


byte
ITG3200::getClockSource() {
    readmem(PWR_MGM, 1, &buff[0]);
    return (buff[0] & PWRMGM_CLK_SEL);
}


void
ITG3200::setClockSource(byte _CLKsource) {
    readmem(PWR_MGM, 1, &buff[0]);
    writemem(PWR_MGM, ((buff[0] & ~PWRMGM_CLK_SEL) | _CLKsource));
}


void
ITG3200::writemem(uint8_t address, uint8_t val) {
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


void
ITG3200::readmem(uint8_t address, uint8_t length, uint8_t buffer[]) {
    // sends register address to read from
    HAL_StatusTypeDef result;
    result = HAL_I2C_Master_Transmit(pHi2c,
                            (uint16_t)dev_address,
                            (uint8_t*)&address,
                            1,
                            10);
    if(result != HAL_OK)
        Error_Handler();
    // receive data
    result = HAL_I2C_Master_Receive(pHi2c,
                                    (uint16_t)dev_address,
                                    (uint8_t *)buffer,
                                     length,
                                     10);
    if(result != HAL_OK)
        Error_Handler();
}

