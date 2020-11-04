/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_SPI

#include "drivers/io.h"
#include "drivers/time.h"
#include "rx/rx_spi_common.h"
#include "rx/rx_spi.h"

static IO_t ledPin;
static bool ledInversion = false;

static IO_t bindPin;
static bool bindRequested;
static bool lastBindPinStatus;

void rxSpiCommonIOInit(const rxConfig_t *rxConfig)
{
    UNUSED(rxConfig);
#ifdef RX_SPI_LED_PIN
    ledPin = IOGetByTag(IO_TAG(RX_SPI_LED_PIN));
    IOInit(ledPin, OWNER_LED, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(ledPin, IOCFG_OUT_PP);
    //#DeXmas #TODO
#ifdef RX_SPI_LED_INVERTED
    ledInversion = true;
#else
    ledInversion = false;
#endif
    rxSpiLedOff();
#else
    ledPin = IO_NONE;
#endif

#ifdef RX_SPI_BIND_PIN
    bindPin = IOGetByTag(IO_TAG(RX_SPI_BIND_PIN));
    IOInit(bindPin, OWNER_RX_SPI_BIND, RESOURCE_OUTPUT, 0);
    IOConfigGPIO(bindPin, IOCFG_IPU);
    lastBindPinStatus = IORead(bindPin);
#else
    bindPin = IO_NONE;
#endif
}

void rxSpiLedOn(void)
{
    if (ledPin) {
        ledInversion ? IOLo(ledPin) : IOHi(ledPin);
    }
}

void rxSpiLedOff(void)
{
    if (ledPin) {
        ledInversion ? IOHi(ledPin) : IOLo(ledPin);
    }
}

void rxSpiLedToggle(void)
{
    if (ledPin) {
        IOToggle(ledPin);
    }
}

void rxSpiLedBlink(timeMs_t blinkMs)
{
    static timeMs_t ledBlinkMs = 0;

    if ((ledBlinkMs + blinkMs) > millis()) {
        return;
    }
    ledBlinkMs = millis();

    rxSpiLedToggle();
}

void rxSpiLedBlinkRxLoss(rx_spi_received_e result)
{
    static timeMs_t rxLossMs = 0;

    if (ledPin) {
        if (result == RX_SPI_RECEIVED_DATA) {
            rxSpiLedOn();
        } else {
            if ((rxLossMs + INTERVAL_RX_LOSS_MS) > millis()) {
                return;
            }
            rxSpiLedToggle();
        }
        rxLossMs = millis();
    }
}

void rxSpiLedBlinkBind(void)
{
    rxSpiLedBlink(INTERVAL_RX_BIND_MS);
}

void rxSpiBind(void)
{
    bindRequested = true;
}

bool rxSpiCheckBindRequested(bool reset)
{
    if (bindPin) {
        bool bindPinStatus = IORead(bindPin);
        if (lastBindPinStatus && !bindPinStatus) {
            bindRequested = true;
        }
        lastBindPinStatus = bindPinStatus;
    }

    if (!bindRequested) {
        return false;
    } else {
        if (reset) {
            bindRequested = false;
        }

        return true;
    }
}
#endif
