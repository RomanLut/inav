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

#include <stdbool.h>
#include <platform.h>
#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/timer.h"
#include "drivers/bus.h"
#include "drivers/pwm_mapping.h"
#include "fc/fc_msp_box.h"
#include "io/piniobox.h"

const timerHardware_t timerHardware[] = {

    DEF_TIM(TIM9, CH1, PA2,   TIM_USE_ANY,   0, 0), //Board TX2 pin, SoftSerial TX   Timer: 2,3 or 5,3 or 9,1

    DEF_TIM(TIM1, CH1, PA8,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 1), // S1_OUT   Timer: 1,1
    DEF_TIM(TIM1, CH2, PA9,  TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 1), // S2_OUT   Timer: 1,2
//    DEF_TIM(TIM1, CH3, PA10, TIM_USE_MC_MOTOR | TIM_USE_FW_SERVO,  0, 0), // S3_OUT   Timer: 1,3
    DEF_TIM(TIM3, CH3, PB0,  TIM_USE_MC_MOTOR | TIM_USE_FW_MOTOR,  0, 0), // S4_OUT   Timer: 1,2 or 3,3

    DEF_TIM(TIM2, CH1, PA15, TIM_USE_ANY,   0, 0), //Board LED pin,  SoftUART2 RX/TX      Timer: 2,1

    DEF_TIM(TIM5, CH2, PA1,  TIM_USE_ANY,   0, 0) //Board CURRENT pin,  SoftSerial1 RX  Timer: 2,2 or 5,2   

//    DEF_TIM(TIM5, CH4, PA3,  TIM_USE_PPM,   0, 1), // PPM on RX2  Timer: 2,4 or 5,4 or 9,2   
//    DEF_TIM(TIM3, CH4, PB1,  TIM_USE_ANY,   0, 0), //Board RSSI PIN,  Timer: 1,3 or 3,4

};

const int timerHardwareCount = sizeof(timerHardware) / sizeof(timerHardware[0]);

void targetConfiguration(void)
{
    pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
}
