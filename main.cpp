#include "mbed.h"
#include "bmi160.h"
#include <cstdint>

// const uint8_t slaveAddr = 0x68;




void dumpImuRegisters(BMI160 &imu);
void printRegister(BMI160 &imu, BMI160::Registers reg);
void printBlock(BMI160 &imu, BMI160::Registers startReg, BMI160::Registers stopReg);
 
 
int main()
{
    //MAX32630FTHR pegasus(MAX32630FTHR::VIO_3V3);
    
    DigitalOut rLED(LED1, LED_OFF);
    DigitalOut gLED(LED2, LED_OFF);
    DigitalOut bLED(LED3, LED_OFF);
    
    I2C i2cBus(P5_7, P6_0);
    i2cBus.frequency(400000);
    BMI160_I2C imu(i2cBus, BMI160_I2C::I2C_ADRS_SDO_LO);
    
    printf("\033[H");  //home
    printf("\033[0J");  //erase from cursor to end of screen
    
    uint32_t failures = 0;
    
    if(imu.setSensorPowerMode(BMI160::GYRO, BMI160::NORMAL) != BMI160::RTN_NO_ERROR)
    {
        printf("Failed to set gyroscope power mode\n");
        failures++;
    }
    wait_us(100000);
    
    if(imu.setSensorPowerMode(BMI160::ACC, BMI160::NORMAL) != BMI160::RTN_NO_ERROR)
    {
        printf("Failed to set accelerometer power mode\n");
        failures++;
    }
    wait_us(100000);
    
    
    BMI160::AccConfig accConfig;
    //example of using getSensorConfig
    if(imu.getSensorConfig(accConfig) == BMI160::RTN_NO_ERROR)
    {
        printf("ACC Range = %d\n", accConfig.range);
        printf("ACC UnderSampling = %d\n", accConfig.us);
        printf("ACC BandWidthParam = %d\n", accConfig.bwp);
        printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    }
    else
    {
        printf("Failed to get accelerometer configuration\n");
        failures++;
    }
    
    //example of setting user defined configuration
    accConfig.range = BMI160::SENS_4G;
    accConfig.us = BMI160::ACC_US_OFF;
    accConfig.bwp = BMI160::ACC_BWP_2;
    accConfig.odr = BMI160::ACC_ODR_8;
    if(imu.setSensorConfig(accConfig) == BMI160::RTN_NO_ERROR)
    {
        printf("ACC Range = %d\n", accConfig.range);
        printf("ACC UnderSampling = %d\n", accConfig.us);
        printf("ACC BandWidthParam = %d\n", accConfig.bwp);
        printf("ACC OutputDataRate = %d\n\n", accConfig.odr);
    }
    else
    {
        printf("Failed to set accelerometer configuration\n");
        failures++;
    }
    
    BMI160::GyroConfig gyroConfig;
    if(imu.getSensorConfig(gyroConfig) == BMI160::RTN_NO_ERROR)
    {
        printf("GYRO Range = %d\n", gyroConfig.range);
        printf("GYRO BandWidthParam = %d\n", gyroConfig.bwp);
        printf("GYRO OutputDataRate = %d\n\n", gyroConfig.odr);
    }
    else
    {
        printf("Failed to get gyroscope configuration\n");
        failures++;
    }
    
    wait_us(100000);
    printf("\033[H");  //home
    printf("\033[0J");  //erase from cursor to end of screen
    
    if(failures == 0)
    {
        float imuTemperature; 
        BMI160::SensorData accData;
        BMI160::SensorData gyroData;
        BMI160::SensorTime sensorTime;
        
        while(1)
        {
            imu.getGyroAccXYZandSensorTime(accData, gyroData, sensorTime, accConfig.range, gyroConfig.range);
            imu.getTemperature(&imuTemperature);
            

            // Original example code.
            /*printf("ACC xAxis = %s%f\n", "\033[K", accData.xAxis.scaled);
            printf("ACC yAxis = %s%f\n", "\033[K", accData.yAxis.scaled);
            printf("ACC zAxis = %s%f\n\n", "\033[K", accData.zAxis.scaled);
            
            printf("GYRO xAxis = %s%f\n", "\033[K", gyroData.xAxis.scaled);
            printf("GYRO yAxis = %s%f\n", "\033[K", gyroData.yAxis.scaled);
            printf("GYRO zAxis = %s%f\n\n", "\033[K", gyroData.zAxis.scaled);
            
            printf("Sensor Time = %s%f\n", "\033[K", sensorTime.seconds);
            printf("Sensor Temperature = %s%f\n", "\033[K", imuTemperature);*/

            printf("ACC xAxis = %s%d\n", "\033[K", accData.xAxis.raw);
            printf("ACC yAxis = %s%d\n", "\033[K", accData.yAxis.raw);
            printf("ACC zAxis = %s%d\n\n", "\033[K", accData.zAxis.raw);
            
            printf("GYRO xAxis = %s%d\n", "\033[K", gyroData.xAxis.raw);
            printf("GYRO yAxis = %s%d\n", "\033[K", gyroData.yAxis.raw);
            printf("GYRO zAxis = %s%d\n\n", "\033[K", gyroData.zAxis.raw);
            
            printf("Sensor Time = %s%f\n", "\033[K", sensorTime.seconds);
            printf("Sensor Temperature = %s%f\n", "\033[K", imuTemperature);
            // my simplified version.
            /*printf("ACC xAxis = %d\n", accData.xAxis.raw);
            printf("ACC yAxis = %d\n", accData.yAxis.raw);
            printf("ACC zAxis = %d\n\n", accData.zAxis.raw);
            
            printf("GYRO xAxis = %d\n", gyroData.xAxis.raw);
            printf("GYRO yAxis = %d\n", gyroData.yAxis.raw);
            printf("GYRO zAxis = %d\n\n", gyroData.zAxis.raw);
            
            printf("Sensor Time = %f\n", sensorTime.seconds);
            printf("Sensor Temperature = %f\n", imuTemperature);*/
            
            printf("\033[H");  //home
            gLED = !gLED;
        }
    }
    else
    {
        while(1)
        {
            rLED = !rLED;
            wait_us(250000);
        }
    }
}
 
 
//*****************************************************************************
void dumpImuRegisters(BMI160 &imu)
{
    printRegister(imu, BMI160::CHIP_ID);
    printBlock(imu, BMI160::ERR_REG,BMI160::FIFO_DATA);
    printBlock(imu, BMI160::ACC_CONF, BMI160::FIFO_CONFIG_1);
    printBlock(imu, BMI160::MAG_IF_0, BMI160::SELF_TEST);
    printBlock(imu, BMI160::NV_CONF, BMI160::STEP_CONF_1);
    printRegister(imu, BMI160::CMD);
    printf("\n");
}
 
 
//*****************************************************************************
void printRegister(BMI160 &imu, BMI160::Registers reg)
{
    uint8_t data;
    if(imu.readRegister(reg, &data) == BMI160::RTN_NO_ERROR)
    {
        printf("IMU Register 0x%02x = 0x%02x\n", reg, data);
    }
    else
    {
        printf("Failed to read register\n");
    }
}
 
 
//*****************************************************************************
void printBlock(BMI160 &imu, BMI160::Registers startReg, BMI160::Registers stopReg)
{
    uint8_t numBytes = ((stopReg - startReg) + 1);
    uint8_t buff[numBytes];
    uint8_t offset = static_cast<uint8_t>(startReg);
    
    if(imu.readBlock(startReg, stopReg, buff) == BMI160::RTN_NO_ERROR)
    {
        for(uint8_t idx = offset; idx < (numBytes + offset); idx++)
        {
            printf("IMU Register 0x%02x = 0x%02x\n", idx, buff[idx - offset]);
        }
    }
    else
    {
        printf("Failed to read block\n");
    }
}