/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

#include <platform.h>

#include "drivers/bus.h"
#include "drivers/io.h"
#include "drivers/pwm_mapping.h"
#include "drivers/timer.h"
#include "drivers/pinio.h"
#include "drivers/sensor.h"

//BUSDEV_REGISTER_SPI_TAG(busdev_mpu6000,     DEVHW_MPU6000,      MPU6000_SPI_BUS,    MPU6000_CS_PIN,     NONE,       0,  DEVFLAGS_NONE,  IMU_MPU6000_ALIGN);
//BUSDEV_REGISTER_SPI_TAG(busdev_icm20689,    DEVHW_ICM20689,     ICM20689_SPI_BUS,   ICM20689_CS_PIN,    NONE,       0,  DEVFLAGS_NONE,  IMU_ICM20689_ALIGN);


timerHardware_t timerHardware[] = {
    DEF_TIM(TIM9,  CH2, PA3,  TIM_USE_PPM,                                                          0, 0 ), // PPM IN
    DEF_TIM(TIM8,  CH2N, PB0,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,                                  0, 0 ), // S1_OUT - DMA1_ST2
    DEF_TIM(TIM8,  CH3N, PB1,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,                                  0, 0 ), // S2_OUT - DMA1_ST4
    DEF_TIM(TIM3,  CH1, PB4,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,                                  0, 0 ), // S3_OUT - DMA1_ST1
    DEF_TIM(TIM2,  CH2, PB3,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,                                  0, 0 ), // S4_OUT - DMA1_ST6
    DEF_TIM(TIM3,  CH4, PC9,  TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO,               0, 0 ), // S5_OUT - DMA1_ST7 - LED Control
    DEF_TIM(TIM3,  CH3, PC8,  TIM_USE_MC_MOTOR | TIM_USE_MC_SERVO | TIM_USE_FW_SERVO | TIM_USE_LED, 0, 0 ), // S6_OUT

    DEF_TIM(TIM1, CH1, PA8, TIM_USE_LED, 0, 0), //LED STRIP
    DEF_TIM(TIM4,  CH3, PB8,  TIM_USE_ANY,                                                          0, 0), //ANY
};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);
