/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include "platform.h"
#include "fc/fc_msp_box.h"
#include "fc/config.h"
#include "io/piniobox.h"

void targetConfiguration(void)
{
    pinioBoxConfigMutable()->permanentId[0] = BOX_PERMANENT_ID_USER1;
    pinioBoxConfigMutable()->permanentId[1] = BOX_PERMANENT_ID_USER2;
#if defined(MATEKF405TE_SD_S9_S10_PINIO3_PINIO4)
    pinioBoxConfigMutable()->permanentId[2] = BOX_PERMANENT_ID_USER3;
    pinioBoxConfigMutable()->permanentId[3] = BOX_PERMANENT_ID_USER4;
#endif
    beeperConfigMutable()->pwmMode = true;
}
