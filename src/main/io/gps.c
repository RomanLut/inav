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
#include "sensors/pitotmeter.h"

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
#include "flight/mixer.h"

typedef struct {
    bool                isDriverBased;
    portMode_t          portMode;           // Port mode RX/TX (only for serial based)
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
    { false, MODE_RX, gpsRestartNMEA, &gpsHandleNMEA },
#else
    { false, 0, NULL, NULL },
#endif

    /* UBLOX binary */
#ifdef USE_GPS_PROTO_UBLOX
    { false, MODE_RXTX, &gpsRestartUBLOX, &gpsHandleUBLOX },
#else
    { false, 0, NULL, NULL },
#endif

    /* UBLOX7PLUS binary */
#ifdef USE_GPS_PROTO_UBLOX
    { false, MODE_RXTX, &gpsRestartUBLOX, &gpsHandleUBLOX },
#else
    { false, 0,  NULL, NULL },
#endif

    /* MSP GPS */
#ifdef USE_GPS_PROTO_MSP
    { true, 0, &gpsRestartMSP, &gpsHandleMSP },
#else
    { false, 0, NULL, NULL },
#endif
};

PG_REGISTER_WITH_RESET_TEMPLATE(gpsConfig_t, gpsConfig, PG_GPS_CONFIG, 2);

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
#if defined(USE_MAG)
	return positionEstimationConfig()->allow_gps_fix_estimation && STATE(AIRPLANE) && sensors(SENSOR_GPS) && sensors(SENSOR_BARO) && sensors(SENSOR_MAG) && compassIsHealthy() && ARMING_FLAG(WAS_EVER_ARMED) && STATE(GPS_FIX_HOME);
#else
    return false;
#endif
}

void updateEstimatedGPSFix(void) {

	static uint32_t lastUpdateMs = 0;
	static int32_t estimated_lat = 0;
	static int32_t estimated_lon = 0;
	static int32_t estimated_alt = 0;

	uint32_t t = millis();
	int32_t dt = t - lastUpdateMs;
	lastUpdateMs = t;

	//-----------------
	/*
	debug[0] = gpsSol.velNED[X];
	debug[1] = gpsSol.velNED[Y];
	debug[2] = gpsSol.velNED[Z];

	float speed1 = pidProfile()->fixedWingReferenceAirspeed;

	float velX1 = rMat[0][0] * speed1;
	float velY1 = -rMat[1][0] * speed1;

	debug[6] = velX1;
	debug[7] = velY1;

	if (isEstimatedWindSpeedValid()) {
		velX1 += getEstimatedWindSpeed(X);
		velY1 += getEstimatedWindSpeed(Y);
	}

	debug[3] = velX1;
	debug[4] = velY1;
	*/
	//---------------------

	static int32_t last_lat = 0;
	static int32_t last_lon = 0;
	static int32_t last_alt = 0;

	if (IS_RC_MODE_ACTIVE(BOXGPSOFF))
	{
		gpsSol.fixType = GPS_NO_FIX;
		gpsSol.hdop = 9999;
		gpsSol.numSat = 0;

		//freeze coordinates
		gpsSol.llh.lat = last_lat;
		gpsSol.llh.lon = last_lon;
		gpsSol.llh.alt = last_alt;

		DISABLE_STATE(GPS_FIX);
	}
	else
	{
		last_lat = gpsSol.llh.lat;
		last_lon = gpsSol.llh.lon;
		last_alt = gpsSol.llh.alt;
	}

//debug[0] = gpsSol.llh.lat;
//debug[1] = gpsSol.llh.lon;

	if (STATE(GPS_FIX) || !canEstimateGPSFix()) {
		DISABLE_STATE(GPS_ESTIMATED_FIX);
		estimated_lat = gpsSol.llh.lat;
		estimated_lon = gpsSol.llh.lon;
		estimated_alt = posControl.gpsOrigin.alt + baro.BaroAlt;
		return;
	}
	
	ENABLE_STATE(GPS_ESTIMATED_FIX);

	gpsSol.fixType = GPS_FIX_3D;
	gpsSol.hdop = 99;
	gpsSol.flags.hasNewData = true;
	gpsSol.numSat = 99;

	gpsSol.eph = 100;
	gpsSol.epv = 100;

	gpsSol.flags.validVelNE = 1;
	gpsSol.flags.validVelD = 0;  //do not provide velocity.z
	gpsSol.flags.validEPE = 1;

	float speed = pidProfile()->fixedWingReferenceAirspeed;

#ifdef USE_PITOT
	if (sensors(SENSOR_PITOT) && pitotIsHealthy())
	{
		speed = getAirspeedEstimate();
	}
#endif

	float velX = rMat[0][0] * speed;
	float velY = -rMat[1][0] * speed;
	// here (velX, velY) is estimated horizontal speed without wind influence = airspeed, cm/sec in NEU frame

	if (isEstimatedWindSpeedValid()) {
		velX += getEstimatedWindSpeed(X);
		velY += getEstimatedWindSpeed(Y);
	}
	// here (velX, velY) is estimated horizontal speed with wind influence = ground speed

	if (STATE(LANDING_DETECTED) || ((posControl.navState == NAV_STATE_RTH_LANDING) && (getThrottlePercent() == 0)))
	{
		velX = 0;
		velY = 0;
	}

	estimated_lat += (int32_t)( velX * dt / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * 1000 ) );
	estimated_lon += (int32_t)( velY * dt / (DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR * 1000 * posControl.gpsOrigin.scale) );
	estimated_alt = posControl.gpsOrigin.alt + baro.BaroAlt;

	gpsSol.llh.lat = estimated_lat;
	gpsSol.llh.lon = estimated_lon;
	gpsSol.llh.alt = estimated_alt;
	
	gpsSol.groundSpeed = (int16_t)fast_fsqrtf(velX * velX + velY * velY);

	float groundCourse = atan2_approx(velY, velX); // atan2 returns [-M_PI, M_PI], with 0 indicating the vector points in the X direction
	if (groundCourse < 0) {
		groundCourse += 2 * M_PIf;
	}
	gpsSol.groundCourse = RADIANS_TO_DECIDEGREES(groundCourse);

	gpsSol.velNED[X] = (int16_t)(velX);
	gpsSol.velNED[Y] = (int16_t)(velY);
	gpsSol.velNED[Z] = 0;

	//debug[0] = gpsSol.velNED[X];
	//debug[1] = gpsSol.velNED[Y];
	//debug[2] = gpsSol.velNED[Z];

	//debug[0] = gpsSol.llh.lat;
	//debug[1] = gpsSol.llh.lon;
}



void gpsProcessNewSolutionData(void)
{
    // Set GPS fix flag only if we have 3D fix
    if (gpsSol.fixType == GPS_FIX_3D && gpsSol.numSat >= gpsConfig()->gpsMinSats) {
        ENABLE_STATE(GPS_FIX);
    }
    else {
        /* When no fix available - reset flags as well */
        gpsSol.flags.validVelNE = false;
        gpsSol.flags.validVelD = false;
        gpsSol.flags.validEPE = false;
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
    gpsSetProtocolTimeout(gpsState.baseTimeoutMs);

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

    gpsSol.flags.validVelNE = false;
    gpsSol.flags.validVelD = false;
    gpsSol.flags.validMag = false;
    gpsSol.flags.validEPE = false;
    gpsSol.flags.validTime = false;
}

void gpsPreInit(void)
{
    // Make sure gpsProvider is known when gpsMagDetect is called
    gpsState.gpsConfig = gpsConfig();
    gpsState.baseTimeoutMs = (gpsState.gpsConfig->provider == GPS_NMEA) ? GPS_TIMEOUT*2 : GPS_TIMEOUT;
}

void gpsInit(void)
{
    gpsState.serialConfig = serialConfig();
    gpsState.gpsConfig = gpsConfig();

    gpsStats.errors = 0;
    gpsStats.timeouts = 0;

    // Reset solution, timeout and prepare to start
    gpsResetSolution();
    gpsSetProtocolTimeout(gpsState.baseTimeoutMs);
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
        speed = speed * sin_approx((now % 1000) / 1000.f * M_PIf) * +speed;
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
        gpsSol.flags.validVelNE = true;
        gpsSol.flags.validVelD = true;
        gpsSol.flags.validEPE = true;
        gpsSol.flags.validTime = true;
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

        gpsSetProtocolTimeout(gpsState.baseTimeoutMs);

        gpsSetState(GPS_RUNNING);
        gpsSol.flags.gpsHeartbeat = !gpsSol.flags.gpsHeartbeat;
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

#ifdef USE_SIMULATOR
    if (ARMING_FLAG(SIMULATOR_MODE)) {
        if ( SIMULATOR_HAS_OPTION(HITL_GPS_TIMEOUT))
        {
            gpsSetState(GPS_LOST_COMMUNICATION);
            sensorsClear(SENSOR_GPS);
            gpsStats.timeouts = 5;
            return false;
        }
        else
        {
            gpsSetState(GPS_RUNNING);
            sensorsSet(SENSOR_GPS);
            bool res = gpsSol.flags.hasNewData;
            gpsSol.flags.hasNewData = false;
            return res;
        }
    }
#endif
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
            gpsSetProtocolTimeout(gpsState.baseTimeoutMs);
            gpsSetState(GPS_RUNNING);
        }
        break;

    case GPS_RUNNING:
        // Call GPS protocol thread
        gpsProviders[gpsState.gpsConfig->provider].protocol();

        // Check for GPS timeout
        if ((millis() - gpsState.lastMessageMs) > gpsState.baseTimeoutMs) {
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

bool isGPSHealthy(void)
{
    return true;
}

bool isGPSHeadingValid(void)
{
    return sensors(SENSOR_GPS) && STATE(GPS_FIX) && gpsSol.numSat >= 6 && gpsSol.groundSpeed >= 300;
}

#endif
