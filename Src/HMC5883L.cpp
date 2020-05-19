/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Magnetometer Arduino Library.
Copyright (C) 2011 Love Electronics (loveelectronics.co.uk)

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

 WARNING: THE HMC5883L IS NOT IDENTICAL TO THE HMC5883!
 Datasheet for HMC5883L:
 http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/HMC5883L_3-Axis_Digital_Compass_IC.pdf
 http://c48754.r54.cf3.rackcdn.com/HMC5883L.pdf

*/


#include "HMC5883L.h"
#include <array>
#include "main.h"

extern void Error_Handler(void);


#define NoError         0
#define ErrorCode_1_Num 1
const char* ErrorCode_1 = "Entered scale was not valid, valid gauss values are: 0.88, 1.3, 1.9, 2.5, 4.0, 4.7, 5.6, 8.1";


HMC5883L::HMC5883L() {
    m_Scale = 1;
    pHi2c = nullptr;
    dev_address = HMC5883L_Address << 1;
}


bool
HMC5883L::init(int16_t _address, I2C_HandleTypeDef *_pHi2c) {
    pHi2c       = _pHi2c;
    dev_address = _address << 1;
    return isConnected();
}


bool
HMC5883L::isConnected() {
    uint8_t data = 0;
    Read(IdentityRegister, 1, &data);
    if(data == IdentityRegisterValue)
        return true;
    else
        return false;
}


bool
HMC5883L::isDataReady() {
    uint8_t data = 0;
    Read(StatusRegister, 1, &data);
    return ((data&1) != 0);
}


// the earth’s magnetic magnetic field ranges from 25 to 65 microTeslas
int16_t
HMC5883L::SetScale(int16_t milliGauss) {
    uint8_t regValue = 0x00;
    if(milliGauss == 880) {
        regValue = 0x00;
        m_Scale = 0.73;
    }
    else if(milliGauss == 1300) {
        regValue = 0x01;
        m_Scale = 0.92;
    }
    else if(milliGauss == 1900) {
        regValue = 0x02;
        m_Scale = 1.22;
    }
    else if(milliGauss == 2500) {
        regValue = 0x03;
        m_Scale = 1.52;
    }
    else if(milliGauss == 4000) {
        regValue = 0x04;
        m_Scale = 2.27;
    }
    else if(milliGauss == 4700) {
        regValue = 0x05;
        m_Scale = 2.56;
    }
    else if(milliGauss == 5600) {
        regValue = 0x06;
        m_Scale = 3.03;
    }
    else if(milliGauss == 8100) {
        regValue = 0x07;
        m_Scale = 4.35;
    }
    else
        return ErrorCode_1_Num;

    // Setting is in the top 3 bits of the register.
    regValue = regValue << 5;
    Write(ConfigurationRegisterB, regValue);
    return NoError;
}


int16_t
HMC5883L::SetMeasurementMode(uint8_t mode) {
    Write(ModeRegister, mode);
    return NoError;
}


void
HMC5883L::ReadRawAxis(MagnetometerRaw* raw) {
    uint8_t buffer[6];
    Read(DataRegisterBegin, 6, buffer);
    // Attention to the Y and Z order. It is the specified one: see datasheet !!!
    raw->XAxis = (buffer[0] << 8) | buffer[1];
    raw->ZAxis = (buffer[2] << 8) | buffer[3];
    raw->YAxis = (buffer[4] << 8) | buffer[5];
}


void
HMC5883L::ReadScaledAxis(MagnetometerScaled* scaled) {
    MagnetometerRaw raw;
    ReadRawAxis(&raw);
    scaled->XAxis = raw.XAxis * m_Scale;
    scaled->ZAxis = raw.ZAxis * m_Scale;
    scaled->YAxis = raw.YAxis * m_Scale;
}


void
HMC5883L::ReadScaledAxis(float* value) {
    MagnetometerRaw raw;
    ReadRawAxis(&raw);
    value[0] = raw.XAxis * m_Scale;
    value[1] = raw.ZAxis * m_Scale;
    value[2] = raw.YAxis * m_Scale;
}


void
HMC5883L::Write(uint8_t address, uint8_t val) {
    std::array<uint8_t, 2> data{address, val};
    HAL_StatusTypeDef result;
    result = HAL_I2C_Master_Transmit(pHi2c,
                                     (uint16_t)dev_address,
                                     (uint8_t*)data.data(),
                                     data.size(),
                                     10);
    if(result != HAL_OK)
        Error_Handler();
}


void
HMC5883L::Read(uint8_t address, int16_t length, uint8_t* buffer) {
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
                                    (uint8_t *)buffer,
                                    length,
                                    10);
    if(result != HAL_OK)
        Error_Handler();
}


const char*
HMC5883L::GetErrorText(int16_t errorCode) {
    if(ErrorCode_1_Num == errorCode)
        return ErrorCode_1;

    return "Error not defined.";
}
