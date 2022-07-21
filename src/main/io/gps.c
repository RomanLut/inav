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
#include <stdint.h>
#include <ctype.h>
#include <math.h>

#include "platform.h"
#include "build/build_config.h"


#ifdef USE_GPS

#include "build/debug.h"

#include "common/maths.h"
#include "common/axis.h"
#include "common/utils.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/compass/compass.h"
#include "drivers/light_led.h"
#include "drivers/serial.h"
#include "drivers/system.h"
#include "drivers/time.h"

#if defined(USE_FAKE_GPS)
#include "fc/runtime_config.h"
#endif

#include "fc/rc_modes.h"

#include "sensors/sensors.h"
#include "sensors/compass.h"
#include "sensors/barometer.h"

#include "io/serial.h"
#include "io/gps.h"
#include "io/gps_private.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "config/feature.h"

#include "fc/config.h"
#include "fc/runtime_config.h"
#include "fc/settings.h"

#include "flight/imu.h"
#include "flight/wind_estimator.h"
#include "flight/pid.h"

typedef struct {
    bool                isDriverBased;
    portMode_t          portMode;           // Port mode RX/TX (only for serial based)
    bool                hasCompass;         // Has a compass (NAZA)
    void                (*restart)(void);   // Restart protocol driver thread
    void                (*protocol)(void);  // Process protocol driver thread
} gpsProviderDescriptor_t;

// GPS public data
gpsReceiverData_t gpsState;
gpsStatistics_t   gpsStats;
gpsSolutionData_t gpsSol;

// Map gpsBaudRate_e index to baudRate_e
baudRate_e gpsToSerialBaudRate[GPS_BAUDRATE_COUNT] = { BAUD_115200, BAUD_57600, BAUD_38400, BAUD_19200, BAUD_9600, BAUD_230400 };

static gpsProviderDescriptor_t  gpsProviders[GPS_PROVIDER_COUNT] = {
    /* NMEA GPS */
#ifdef USE_GPS_PROTO_NMEA
    { false, MODE_RX, false, &gpsRestartNMEA_MTK, &gpsHandleNMEA },
#else
    { false, 0, false,  NULL, NULL },
#endif

    /* UBLOX binary */
#ifdef USE_GPS_PROTO_UBLOX
    { false, MODE_RXTX, false, &gpsRestartUBLOX, &gpsHandleUBLOX },
#else
    { false, 0, false,  NULL, NULL },
#endif

    /* Stub */
    { false, 0, false,  NULL, NULL },

    /* NAZA GPS module */
#ifdef USE_GPS_PROTO_NAZA
    { false, MODE_RX, true, &gpsRestartNAZA, &gpsHandleNAZA },
#else
    { false, 0, false,  NULL, NULL },
#endif

    /* UBLOX7PLUS binary */
#ifdef USE_GPS_PROTO_UBLOX
    { false, MODE_RXTX, false, &gpsRestartUBLOX, &gpsHandleUBLOX },
#else
    { false, 0, false,  NULL, NULL },
#endif

    /* MTK GPS */
#ifdef USE_GPS_PROTO_MTK
    { false, MODE_RXTX, false, &gpsRestartNMEA_MTK, &gpsHandleMTK },
#else
    { false, 0, false,  NULL, NULL },
#endif

    /* MSP GPS */
#ifdef USE_GPS_PROTO_MSP
    { true, 0, false, &gpsRestartMSP, &gpsHandleMSP },
#else
    { false, 0, false,  NULL, NULL },
#endif
};

PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 0);

PG_RESET_TEMPLATE(gpsConfig_t, gpsConfig,
    .provider = SETTING_GPS_PROVIDER_DEFAULT,
    .sbasMode = SETTING_GPS_SBAS_MODE_DEFAULT,
    .autoConfig = SETTING_GPS_AUTO_CONFIG_DEFAULT,
    .autoBaud = SETTING_GPS_AUTO_BAUD_DEFAULT,
    .dynModel = SETTING_GPS_DYN_MODEL_DEFAULT,
    .gpsMinSats = SETTING_GPS_MIN_SATS_DEFAULT,
    .ubloxUseGalileo = SETTING_GPS_UBLOX_USE_GALILEO_DEFAULT
);

void gpsSetState(gpsState_e state)
{
    gpsState.state = state;
    gpsState.lastStateSwitchMs = millis();
}

static void gpsUpdateTime(void)
{
    if (!rtcHasTime() && gpsSol.flags.validTime && gpsSol.time.year != 0) {
        rtcSetDateTime(&gpsSol.time);
    }
}

void gpsSetProtocolTimeout(timeMs_t timeoutMs)
{
    gpsState.lastLastMessageMs = gpsState.lastMessageMs;
    gpsState.lastMessageMs = millis();
    gpsState.timeoutMs = timeoutMs;
}


/*
//V in cm/sec
//pitch in decidegrees
uint16_t VbyPitch(uint16_t v, int16_t pitch) {
	if (pitch > 200) pitch = 200;
	if (pitch < -200) pitch = -200;
	pitch *= 11;
	return v > pitch ? v + pitch : 0;
}

//throttle  0..100
uint16_t VbyThrottle(uint16_t throttle) {
	if (!ARMING_FLAG(ARMED)) return 0;
	if (baro.BaroAlt < 500) return 0;
	if (throttle < 20) throttle = 20;
	return (uint32_t)(throttle - 20) * 102 * 28 / 80 + 63 * 28;
}
*/

bool canEstimateGPSFix(void)
{
	return positionEstimationConfig()->allow_gps_fix_estimation && STATE(AIRPLANE) && sensors(SENSOR_GPS) && sensors(SENSOR_BARO) && sensors(SENSOR_MAG) && ARMING_FLAG(WAS_EVER_ARMED);
}

void updateEstimatedGPSFix(void) {

	static uint32_t lastUpdateMs = 0;
	static uint32_t estimated_lat = 0;
	static uint32_t estimated_lon = 0;

	uint32_t t = millis();
	uint32_t dt = t - lastUpdateMs;
	lastUpdateMs = t;

	if (IS_RC_MODE_ACTIVE(BOXGPSOFF))
	{
		gpsSol.fixType = GPS_NO_FIX;
		gpsSol.hdop = 9999;
		gpsSol.numSat = 0;
		DISABLE_STATE(GPS_FIX);
	}

	if (STATE(GPS_FIX)) {
		DISABLE_STATE(GPS_ESTIMATED_FIX);
		lastUpdateMs = t;
		estimated_lat = gpsSol.llh.lat;
		estimated_lon = gpsSol.llh.lon;
		return;
	}

	if (!canEstimateGPSFix()) return;
	
	ENABLE_STATE(GPS_ESTIMATED_FIX);

	gpsSol.fixType = GPS_FIX_3D;
	gpsSol.hdop = 99;
	gpsSol.flags.hasNewData = true;
	gpsSol.numSat = 99;

	gpsSol.flags.validVelNE = 1;
	gpsSol.flags.validVelD = 1;
	gpsSol.flags.validEPE = 1;

	float speed = pidProfile()->fixedWingReferenceAirspeed;

	float speedV[XYZ_AXIS_COUNT]; 
	speedV[X] = rMat[0][0] * speed;
	speedV[Y] = -rMat[1][0] * speed;
	speedV[Z] = -rMat[2][0] * speed;
	// here speedV[] is estimated speed without wind influence, cm/sec in NEU frame

	if (isEstimatedWindSpeedValid()) {
		speedV[X] += getEstimatedWindSpeed(X);
		speedV[Y] += getEstimatedWindSpeed(Y);
		speedV[Z] += getEstimatedWindSpeed(Z);
	}
	// here speedV[] is estimated speed with wind influence

	if (STATE(LANDING_DETECTED))
	{
		speedV[X] = 0;
		speedV[Y] = 0;
		speedV[Z] = 0;
	}

	estimated_lat += speedV[X] * dt / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR / 1000;
	estimated_lon += speedV[Y] * dt / DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR / 1000;

	gpsSol.llh.lat = estimated_lat;
	gpsSol.llh.lon = estimated_lon;
	gpsSol.llh.alt = posControl.gpsOrigin.alt + baro.BaroAlt;
	
	gpsSol.groundSpeed = (int16_t)fast_fsqrtf(speedV[X]* speedV[X] + speedV[Y]* speedV[Y]);

	float groundCourse = atan2_approx(speedV[Y], speedV[X]); // atan2 returns [-M_PI, M_PI], with 0 indicating the vector points in the X direction
	if (groundCourse < 0) {
		groundCourse += 2 * M_PIf;
	}
	gpsSol.groundCourse = RADIANS_TO_DECIDEGREES(groundCourse);

	gpsSol.velNED[X] = (int16_t)(speedV[X]);
	gpsSol.velNED[Y] = (int16_t)(speedV[Y]);
	gpsSol.velNED[Z] = (int16_t)(speedV[Z]);

	gpsSol.eph = 100;
	gpsSol.epv = 100;
}



void gpsProcessNewSolutionData(void)
{
    // Set GPS fix flag only if we have 3D fix
    if (gpsSol.fixType == GPS_FIX_3D && gpsSol.numSat >= gpsConfig()->gpsMinSats) {
        ENABLE_STATE(GPS_FIX);
    }
    else {
        /* When no fix available - reset flags as well */
        gpsSol.flags.validVelNE = 0;
        gpsSol.flags.validVelD = 0;
        gpsSol.flags.validEPE = 0;
        DISABLE_STATE(GPS_FIX);
    }

    // Set sensor as ready and available
    sensorsSet(SENSOR_GPS);

	updateEstimatedGPSFix();

    // Pass on GPS update to NAV and IMU
    onNewGPSData();

    // Update time
    gpsUpdateTime();

    // Update timeout
    gpsSetProtocolTimeout(GPS_TIMEOUT);

    // Update statistics
    gpsStats.lastMessageDt = gpsState.lastMessageMs - gpsState.lastLastMessageMs;
    gpsSol.flags.hasNewData = true;

    // Toggle heartbeat
    gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;
}

static void gpsResetSolution(void)
{
    gpsSol.eph = 9999;
    gpsSol.epv = 9999;
    gpsSol.numSat = 0;
    gpsSol.hdop = 9999;

    gpsSol.fixType = GPS_NO_FIX;

    gpsSol.flags.validVelNE = 0;
    gpsSol.flags.validVelD = 0;
    gpsSol.flags.validMag = 0;
    gpsSol.flags.validEPE = 0;
    gpsSol.flags.validTime = 0;
}

void gpsPreInit(void)
{
    // Make sure gpsProvider is known when gpsMagDetect is called
    gpsState.gpsConfig = gpsConfig();
}

void gpsInit(void)
{
    gpsState.serialConfig = serialConfig();
    gpsState.gpsConfig = gpsConfig();

    gpsStats.errors = 0;
    gpsStats.timeouts = 0;

    // Reset solution, timeout and prepare to start
    gpsResetSolution();
    gpsSetProtocolTimeout(GPS_TIMEOUT);
    gpsSetState(GPS_UNKNOWN);

    // If given GPS provider has protocol() function not defined - we can't use it
    if (!gpsProviders[gpsState.gpsConfig->provider].protocol) {
        featureClear(FEATURE_GPS);
        return;
    }

    // Shortcut for driver-based GPS (MSP)
    if (gpsProviders[gpsState.gpsConfig->provider].isDriverBased) {
        gpsSetState(GPS_INITIALIZING);
        return;
    }

    serialPortConfig_t * gpsPortConfig = findSerialPortConfig(FUNCTION_GPS);
    if (!gpsPortConfig) {
        featureClear(FEATURE_GPS);
        return;
    }

    // Start with baud rate index as configured for serial port
    int baudrateIndex;
    gpsState.baudrateIndex = GPS_BAUDRATE_115200;
    for (baudrateIndex = 0, gpsState.baudrateIndex = 0; baudrateIndex < GPS_BAUDRATE_COUNT; baudrateIndex++) {
        if (gpsToSerialBaudRate[baudrateIndex] == gpsPortConfig->gps_baudrateIndex) {
            gpsState.baudrateIndex = baudrateIndex;
            break;
        }
    }

    // Start with the same baud for autodetection
    gpsState.autoBaudrateIndex = gpsState.baudrateIndex;

    // Open serial port
    portMode_t mode = gpsProviders[gpsState.gpsConfig->provider].portMode;
    gpsState.gpsPort = openSerialPort(gpsPortConfig->identifier, FUNCTION_GPS, NULL, NULL, baudRates[gpsToSerialBaudRate[gpsState.baudrateIndex]], mode, SERIAL_NOT_INVERTED);

    // Check if we have a serial port opened
    if (!gpsState.gpsPort) {
        featureClear(FEATURE_GPS);
        return;
    }

    gpsSetState(GPS_INITIALIZING);
}

#ifdef USE_FAKE_GPS
static bool gpsFakeGPSUpdate(void)
{
#define FAKE_GPS_INITIAL_LAT 509102311
#define FAKE_GPS_INITIAL_LON -15349744
#define FAKE_GPS_GROUND_ARMED_SPEED 350 // In cm/s
#define FAKE_GPS_GROUND_UNARMED_SPEED 0
#define FAKE_GPS_GROUND_COURSE_DECIDEGREES 300 //30deg

    // Each degree in latitude corresponds to 111km.
    // Each degree in longitude at the equator is 111km,
    // going down to zero as latitude gets close to 90º.
    // We approximate it linearly.

    static int32_t lat = FAKE_GPS_INITIAL_LAT;
    static int32_t lon = FAKE_GPS_INITIAL_LON;

    timeMs_t now = millis();
    uint32_t delta = now - gpsState.lastMessageMs;
    if (delta > 100) {
        int32_t speed = ARMING_FLAG(ARMED) ? FAKE_GPS_GROUND_ARMED_SPEED : FAKE_GPS_GROUND_UNARMED_SPEED;
        int32_t cmDelta = speed * (delta / 1000.0f);
        int32_t latCmDelta = cmDelta * cos_approx(DECIDEGREES_TO_RADIANS(FAKE_GPS_GROUND_COURSE_DECIDEGREES));
        int32_t lonCmDelta = cmDelta * sin_approx(DECIDEGREES_TO_RADIANS(FAKE_GPS_GROUND_COURSE_DECIDEGREES));
        int32_t latDelta = ceilf((float)latCmDelta / (111 * 1000 * 100 / 1e7));
        int32_t lonDelta = ceilf((float)lonCmDelta / (111 * 1000 * 100 / 1e7));
        if (speed > 0 && latDelta == 0 && lonDelta == 0) {
            return false;
        }
        lat += latDelta;
        lon += lonDelta;
        gpsSol.fixType = GPS_FIX_3D;
        gpsSol.numSat = 6;
        gpsSol.llh.lat = lat;
        gpsSol.llh.lon = lon;
        gpsSol.llh.alt = 0;
        gpsSol.groundSpeed = speed;
        gpsSol.groundCourse = FAKE_GPS_GROUND_COURSE_DECIDEGREES;
        gpsSol.velNED[X] = speed * cos_approx(DECIDEGREES_TO_RADIANS(FAKE_GPS_GROUND_COURSE_DECIDEGREES));
        gpsSol.velNED[Y] = speed * sin_approx(DECIDEGREES_TO_RADIANS(FAKE_GPS_GROUND_COURSE_DECIDEGREES));
        gpsSol.velNED[Z] = 0;
        gpsSol.flags.validVelNE = 1;
        gpsSol.flags.validVelD = 1;
        gpsSol.flags.validEPE = 1;
        gpsSol.flags.validTime = 1;
        gpsSol.eph = 100;
        gpsSol.epv = 100;
        gpsSol.time.year = 1983;
        gpsSol.time.month = 1;
        gpsSol.time.day = 1;
        gpsSol.time.hours = 3;
        gpsSol.time.minutes = 15;
        gpsSol.time.seconds = 42;

        ENABLE_STATE(GPS_FIX);
        sensorsSet(SENSOR_GPS);
        gpsUpdateTime();
        onNewGPSData();

        gpsSetProtocolTimeout(GPS_TIMEOUT);

        gpsSetState(GPS_RUNNING);
        return true;
    }
    return false;
}
#endif

uint16_t gpsConstrainEPE(uint32_t epe)
{
    return (epe > 9999) ? 9999 : epe; // max 99.99m error
}

uint16_t gpsConstrainHDOP(uint32_t hdop)
{
    return (hdop > 9999) ? 9999 : hdop; // max 99.99m error
}

bool gpsUpdate(void)
{
    // Sanity check
    if (!feature(FEATURE_GPS)) {
        sensorsClear(SENSOR_GPS);
        DISABLE_STATE(GPS_FIX);
        return false;
    }

    /* Extra delay for at least 2 seconds after booting to give GPS time to initialise */
    if (!isMPUSoftReset() && (millis() < GPS_BOOT_DELAY)) {
        sensorsClear(SENSOR_GPS);
        DISABLE_STATE(GPS_FIX);
        return false;
    }

    if (ARMING_FLAG(SIMULATOR_MODE)) {
        gpsUpdateTime();
        gpsSetState(GPS_RUNNING);
		sensorsSet(SENSOR_GPS);
		return gpsSol.flags.hasNewData;
    }
#ifdef USE_FAKE_GPS
    return gpsFakeGPSUpdate();
#else

    // Assume that we don't have new data this run
    gpsSol.flags.hasNewData = false;

    switch (gpsState.state) {
    default:
    case GPS_INITIALIZING:
        // Wait for GPS_INIT_DELAY before starting the GPS protocol thread
        if ((millis() - gpsState.lastStateSwitchMs) >= GPS_INIT_DELAY) {
            // Reset internals
            DISABLE_STATE(GPS_FIX);
            gpsSol.fixType = GPS_NO_FIX;

            // Reset solution
            gpsResetSolution();

            // Call GPS protocol reset handler
            gpsProviders[gpsState.gpsConfig->provider].restart();

            // Switch to GPS_RUNNING state (mind the timeout)
            gpsSetProtocolTimeout(GPS_TIMEOUT);
            gpsSetState(GPS_RUNNING);
        }
        break;

    case GPS_RUNNING:
        // Call GPS protocol thread
        gpsProviders[gpsState.gpsConfig->provider].protocol();

        // Check for GPS timeout
        if ((millis() - gpsState.lastMessageMs) > GPS_TIMEOUT) {
            sensorsClear(SENSOR_GPS);
            DISABLE_STATE(GPS_FIX);
            gpsSol.fixType = GPS_NO_FIX;
            gpsSetState(GPS_LOST_COMMUNICATION);
        }
        break;

    case GPS_LOST_COMMUNICATION:
        gpsStats.timeouts++;
        gpsSetState(GPS_INITIALIZING);
        break;
    }

    return gpsSol.flags.hasNewData;
#endif
}

void gpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(gpsState.gpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if (!(gpsState.gpsPort->mode & MODE_TX))
    serialSetMode(gpsState.gpsPort, gpsState.gpsPort->mode | MODE_TX);

    LED0_OFF;
    LED1_OFF;

    char c;
    while (1) {
        if (serialRxBytesWaiting(gpsState.gpsPort)) {
            LED0_ON;
            c = serialRead(gpsState.gpsPort);
            serialWrite(gpsPassthroughPort, c);
            LED0_OFF;
        }
        if (serialRxBytesWaiting(gpsPassthroughPort)) {
            LED1_ON;
            c = serialRead(gpsPassthroughPort);
            serialWrite(gpsState.gpsPort, c);
            LED1_OFF;
        }
    }
}

void updateGpsIndicator(timeUs_t currentTimeUs)
{
    static timeUs_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (gpsSol.numSat>= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED1_TOGGLE;
    }
}

/* Support for built-in magnetometer accessible via the native GPS protocol (i.e. NAZA) */
bool gpsMagInit(magDev_t *magDev)
{
    UNUSED(magDev);
    return true;
}

bool gpsMagRead(magDev_t *magDev)
{
    magDev->magADCRaw[X] = gpsSol.magData[0];
    magDev->magADCRaw[Y] = gpsSol.magData[1];
    magDev->magADCRaw[Z] = gpsSol.magData[2];
    return gpsSol.flags.validMag;
}

bool gpsMagDetect(magDev_t *mag)
{
    if (!(feature(FEATURE_GPS) && gpsProviders[gpsState.gpsConfig->provider].hasCompass)) {
        return false;
    }

    if (!gpsProviders[gpsState.gpsConfig->provider].protocol || !findSerialPortConfig(FUNCTION_GPS)) {
        return false;
    }

    mag->init = gpsMagInit;
    mag->read = gpsMagRead;
    return true;
}

bool isGPSHealthy(void)
{
    return true;
}

bool isGPSHeadingValid(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 6 && gpsSol.groundSpeed >= 300;
}

#endif
