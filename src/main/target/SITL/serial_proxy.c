/*
 * This file is part of INAV Project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this file,
 * You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Alternatively, the contents of this file may be used under the terms
 * of the GNU General Public License Version 3, as described below:
 *
 * This file is free software: you may copy, redistribute and/or modify
 * it under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or (at your
 * option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */


#include "serial_proxy.h"

#if defined(SITL_BUILD)

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <time.h>
#include <unistd.h>

#include "platform.h"


#include <sys/types.h>
#include <fcntl.h>
#include <errno.h> 
#include <termios.h> 
#include <unistd.h>
#include <unistd.h>

#include "drivers/serial_tcp.h"

int serialUartIndex = -1;
int serialPortIndex = -1;
int serialBaudRate = 115200;
OptSerialStopBits_e serialStopBits = OPT_SERIAL_STOP_BITS_ONE;  //0:None|1:One|2:OnePointFive|3:Two 
OptSerialParity_e serialParity = OPT_SERIAL_PARITY_NONE;
bool serialFCProxy = false;

#define SERIAL_BUFFER_SIZE 256

#if defined(__CYGWIN__)
#include <windows.h>
static HANDLE hSerial;
#else
static int fd;
#endif
static bool connected = false;

void serialProxyInit(void) {
    if (( serialUartIndex == -1 ) || (serialPortIndex==-1)) {
        return;
    }
    connected = false;

    char portName[20];
#if defined(__CYGWIN__)
    sprintf(portName, "\\\\.\\COM%d", serialPortIndex );

    hSerial = CreateFile(portName,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        if (GetLastError() == ERROR_FILE_NOT_FOUND) {
            fprintf(stderr, "[SERIALPROXY] ERROR: COM port was not attached. Reason: %s not available.", portName);
        } else {
            fprintf(stderr, "[SERIALPROXY] Cand not connect to COM port, unknown error.");
        }
        return;
    } else {
        DCB dcbSerialParams = { 0 };
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            fprintf(stderr, "[SERIALPROXY] failed to get current serial parameters!");
        } else {
            dcbSerialParams.BaudRate = serialBaudRate;
            dcbSerialParams.ByteSize = 8;

            switch (serialStopBits) {
                case OPT_SERIAL_STOP_BITS_ONE:
                    dcbSerialParams.StopBits = ONESTOPBIT;
                    break;
                case OPT_SERIAL_STOP_BITS_TWO:
                    dcbSerialParams.StopBits = TWOSTOPBITS;
                    break;
                case OPT_SERIAL_STOP_BITS_INVALID:
                    break;
            }

            switch (serialParity) {
                case OPT_SERIAL_PARITY_EVEN:
                    dcbSerialParams.Parity = EVENPARITY;
                    break;
                case OPT_SERIAL_PARITY_NONE:
                    dcbSerialParams.Parity = NOPARITY;
                    break;
                case OPT_SERIAL_PARITY_ODD:
                    dcbSerialParams.Parity = ODDPARITY;
                    break;
                case OPT_SERIAL_PARITY_INVALID:
                    break;
            }    

            if (!SetCommState(hSerial, &dcbSerialParams)) {
                fprintf(stderr, "[SERIALPROXY] ALERT: Could not set Serial Port parameters\n");
            } else {
                COMMTIMEOUTS comTimeOut;
                comTimeOut.ReadIntervalTimeout = MAXDWORD;
                comTimeOut.ReadTotalTimeoutMultiplier = 0;
                comTimeOut.ReadTotalTimeoutConstant = 0;
                comTimeOut.WriteTotalTimeoutMultiplier = 0;
                comTimeOut.WriteTotalTimeoutConstant = 300;
                SetCommTimeouts(hSerial, &comTimeOut);
            }
        }
    }
#else
    sprintf(portName, "/dev/ttyACM%d", serialPortIndex);

    fd = open(portName, O_RDWR);
    if (fd == -1)
    {
        fprintf(stderr, "[SERIALPROXY] Can not connect to COM port %s\n", portName);
        return;
    }

    struct termios terminalOptions;
    memset(&terminalOptions, 0, sizeof(struct termios));
    tcgetattr(fd, &terminalOptions);

    cfmakeraw(&terminalOptions);

    cfsetispeed(&terminalOptions, serialBaudRate);
    cfsetospeed(&terminalOptions, serialBaudRate);

    terminalOptions.c_cflag = CREAD | CLOCAL;
    terminalOptions.c_cflag |= CS8;
    terminalOptions.c_cflag &= ~HUPCL;

    terminalOptions.c_lflag &= ~ICANON;
    terminalOptions.c_lflag &= ~ECHO; // Disable echo
    terminalOptions.c_lflag &= ~ECHOE; // Disable erasure
    terminalOptions.c_lflag &= ~ECHONL; // Disable new-line echo
    terminalOptions.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

    terminalOptions.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    terminalOptions.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

    terminalOptions.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    terminalOptions.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    terminalOptions.c_cc[VMIN] = 0;
    terminalOptions.c_cc[VTIME] = 0;

    //https://stackoverflow.com/questions/46905431/linux-serial-port-1-5-stop-bits
    switch (serialStopBits) {
        case OPT_SERIAL_STOP_BITS_ONE:
            terminalOptions.c_cflag &= ~CSTOPB;  
            break;
        case OPT_SERIAL_STOP_BITS_TWO:
            terminalOptions.c_cflag |= CSTOPB;
            break;
        case OPT_SERIAL_STOP_BITS_INVALID:
            break;
    }

    switch (serialParity) {
        case OPT_SERIAL_PARITY_EVEN:
            terminalOptions.c_cflag |= PARENB;
            terminalOptions.c_cflag &= ~PARODD;
            break;
        case OPT_SERIAL_PARITY_NONE:
            terminalOptions.c_cflag &= ~PARENB;
            terminalOptions.c_cflag &= ~PARODD;
            break;
        case OPT_SERIAL_PARITY_ODD:
            terminalOptions.c_cflag |= PARENB;
            terminalOptions.c_cflag |= PARODD;
            break;
        case OPT_SERIAL_PARITY_INVALID:
            break;
    }    

    int ret = tcsetattr(fd, TCSANOW, &terminalOptions); 
    if (ret == -1)
    {
        fprintf(stderr, "[SERIALPROXY] Failed to configure device: %s\n", portName);
        perror("tcsetattr");
        return;
    }
#endif
    connected = true;

    fprintf(stderr, "[SERIALPROXY] connected %s to UART%d\n", portName, serialUartIndex);
}

void serialProxyClose(void) {
    if (connected) {
        connected = false;
#if defined(__CYGWIN__)
        CloseHandle(hSerial);
#else
        close(fd);
#endif
    }
}

int serialProxyReadData(unsigned char *buffer, unsigned int nbChar) {
    if (!connected) return 0;

#if defined(__CYGWIN__)
    COMSTAT status;
    DWORD errors;
    DWORD bytesRead;

    ClearCommError(hSerial, &errors, &status);
    if (status.cbInQue>0) {
        unsigned int toRead = (status.cbInQue>nbChar) ? nbChar : status.cbInQue;
        if (ReadFile(hSerial, buffer, toRead, &bytesRead, NULL) && (bytesRead != 0)) {
            return bytesRead;
        }
    }
    return 0;
#else
    if (nbChar == 0) return 0;
    int bytesRead = read(fd, buffer, nbChar);
    return bytesRead;
#endif    
}

bool serialProxyWriteData(unsigned char *buffer, unsigned int nbChar) {
  if (!connected) return false;

#if defined(__CYGWIN__)
        COMSTAT status;
        DWORD errors;
        DWORD bytesSent;
        if (!WriteFile(hSerial, (void *)buffer, nbChar, &bytesSent, 0)) {
            ClearCommError(hSerial, &errors, &status);
            return false;
        }
#else
        ssize_t l = write(fd, buffer, nbChar);
        if ( l!= nbChar ) {
            reurn false;
        }
#endif    
    return true;
}

bool serialProxyIsConnected(void) {
    return connected;
}

extern void serialProxyProcess(void) {

    if (( serialUartIndex == -1 ) || (!connected)) return;

    unsigned char buf[SERIAL_BUFFER_SIZE];

    uint32_t avail = tcpRXBytesFree(serialUartIndex-1);
    if ( avail == 0 ) return;
    if (avail > SERIAL_BUFFER_SIZE) avail = SERIAL_BUFFER_SIZE;

    int count = serialProxyReadData(buf, avail);
    if (count == 0) return;

    tcpReceiveBytesEx( serialUartIndex-1, buf, count);
}

#endif