//
//  wsc32.c    VERSION 6.0.0
//
//  Windows Standard Serial Communications Library
//  Copyright (C) MarshallSoft Computing, Inc. 1997-2017.
//
//  2.0.0 01/28/97 Original version
//  2.0.1 03/09/97 wsc.h & makefile changed.
//                 SioParms & SioFlow return 0 if OK.
//                 SioDTR & SioRTS return WSC_RANGE if bad parameter
//  2.1.0 05/07/97 Added "Expires" option to SioInfo.
//        05/17/97 Added SioRead.
//  2.1.1 06/03/97 Added args to SioWinErr to return error text.
//  2.1.2 08/18/97 Increased to 16 ports.
//  2.2.0 08/26/97 Added dllimport & dllexport to wsc.h
//        09/06/97 Changed XonLim and XoffLim values.
//  2.2.1 03/03/98 First arg of SioRead() changed to UART base address.
//  2.2.2 04/08/98 DTR & RTS flow control disabled at startup.
//  2.3.0 06/24/98 Only NBR_PORTS must be modified to change # of ports.
//        06/29/98 Added SioTimer() function.
//        06/29/98 SioBaud & SioParms can be called before SioReset.
//        07/12/98 SioRead uses default port address for COM1 thru COM4.
//  2.3.1 07/25/98 Changed WSC_VERSION to 3 hex digits.
//  2.3.2 01/08/99 Increased NBR_PORTS to 20.
//  2.3.3 01/17/99 Made ComText an array.
//  2.3.4 01/26/99 Fixed detection of BREAK in SioBrkSig.
//  2.4.0 03/17/99 Added SioEvent function (WIN32 only).
//        03/17/99 Added SioReset(-1, DTR_Default, RTS_Default).
//  2.4.1 08/13/99 SioEvent returns 1 on success.
//  2.4.2 09/22/99 Ran PC-Lint against source. Source modified.
//  2.4.3 09/28/99 Let # ports to be set from the compile line.
//  2.4.4 01/25/00 Check return code from SetupComm().
//  3.0.0 07/10/00 All constants (wsc.h1 & wsc.h2) begin with "WSC_".
//                 Added SioMessage, which uses win32 thread to send message.
//                 NBR_PORTS set to 32.
//  3.0.1 07/24/00 SioClose & SioMessage terminates running thread, if any.
//  3.1.0 10/25/00 Use thread to allow SioPutc and SioPuts to return immediately.
//  3.1.1 11/29/00 SioDebug('X') prevents calling of RESETDEV.
//  3.1.2 02/23/01 SioDebug returns argument (Parm) if recognized.
//  3.1.3 04/03/01 Modified so can be compiled for Windows CE [USE_WIN_CE].
//                 Default for RESETDEV is "not called". SioDebug('R') to enable.
//                 SioGetc & SioGets zero unused bits (DataBits 5,6,7).
//                 Corrected problem with SioBaud(-1, BaudRateCode).
//  3.1.4 04/27/01 SioDebug returns -1 if no match.
//  3.1.5 05/03/01 Added SioDebug('W') toggle.
//  3.1.6 05/25/01 Added code to detect active threads & to close thread handles.
//  3.1.7 02/07/02 Added USE_THREADS, so can compile version of WSC32.C without threads.
//                 Added SetThreadPriority() [must uncomment]
//  3.1.8 03/26/02 Comm handle not saved in SioReset unless good.
//  3.1.9 03/29/02 SioEvent returns mask that caused event.
//  3.1.10 5/02/02 Added anti-crack code to SioReset.
//  3.2.0 07/15/02 Replaced WSC_VERSION with 'VerAndBld' string.
//                 Remapped Windows message "The system cannot find the file specified" to
//                 "The system cannot open the port specified".
//  3.2.1 02/26/03 Replaced "(char *)" with "(LPSTR") in above windows message (for WIN/CE only)
//  3.2.2 04/18/03 Port added to PostMessage in EventThread().
//  3.3.0 05/14/03 EventThread renamed to MsgThread.
//                 EventHandle renamed to MsgHandle.
//                 Added SioSetInteger()
//                 Moved 'W' flag to SioSetInteger.
//        07/18/03 SioPutc/SioPuts will not block if SioEvent was called.
//                 USE_WIN_CE removed (see WSC4eVC and WSCe4VB)
//  3.3.1 08/28/03 Added SioSetInteger(Port, 'H', 0)  {returns COM handle}
//        09/12/03 Added SioKeyCode and SioGetReg
//  4.0.0 11/11/03 Added support for VC.Net (DLL not affected)
//  4.0.1 12/03/03 If Windows NT/2000/XP, then kill & re-start EventThread in SioGetc & SioGets
//  4.0.2  1/08/04 Fixed problem with SioTxClear().
//         1/14/04 Change is TrialRun code [SW version only].
//  4.0.3  7/16/04 MAJOR: Added overlapped I/O (for non-Win95) so can signal threads to exit w/o killing them.
//                 Increased default burst size to 256
//  4.0.4  7/29/04 SioFlow returns WSC_RANGE if cannot recognize parameter.
//  4.1.0  8/02/04 Adjusted thread priorities.
//  4.1.1  9/03/04 Call SioSetInteger(Port,'X',1) to ignore return code from SetupComm (problem with some USB/serial drivers)
//  4.1.2 12/01/04 SioFlow returns 1 if OK.
//  4.1.3  1/06/05 SioSetInteger(Port, 'S', 1) always forces SioEvent to unblock.
//  4.1.4  2/21/05 Event mutex code added to EventThread() to prevent race conditions.
//         3/11/05 Message box displays error if SioWinError(Buffer, 0) called.
//  4.1.5  4/06/05 Modifications to trialDisplayNagScreen (TrialRun.inc)
//  4.1.6  4/27/05 Major change in overlapped I/O
//  4.1.7  8/03/05 Removed "if(Size==0) if(Code==WSC_IO_PENDING) return WSC_BUSY" in SioPuts
//  4.1.8  1/12/06 Fixed problem: SioEvent returning wrong code.
//  4.2.0  1/18/06 SioRxClear clears byte saved by SioUnGet
//                 NBR_PORTS increased to maximum of 256.
//                 Added SioEventChar() and SioEventWait() functions.
//  4.2.1  3/03/06 Fixed problem with SioTxQue
//  4.2.2  6/29/06 SioParms checks the range of passed arguments.
//  4.2.3  7/16/06 Port verified in SioEventChar
//  4.2.4  1/05/07 SioStatus returns -1 if port is not functioning (USB/serial port disconnected)
//  4.2.5  3/05/07 SioKeyCode always fails if passed wrong keycode (other than 0)
//  4.2.6  6/11/07 Added SioByteToShort and SioShortToByte (WSC32 only)
//  4.3.0  8/25/07 Minor clean up.
//  4.4.0  1/19/09 Added SioSetTimeouts().
//  5.0.0  11/4/09 Added support for 64-bits (Win64)
//                 SioGetc will timeout if oioReadWait is PENDING after one second
//                 SioReset will not open ports that are not readable
//                 Fixed popup schedule.
//  5.1.0  8/16/11 Added SioRxWait function.
//  5.1.1  1/18/12 Added SioTimeMark()
//  5.2.0  7/01/12 Added SioQuiet() and SioWaitFor().
//  5.2.1  4/10/13 Added SioLRC()
//  5.2.2  9/24/13 SioQuiet and SioWaitFor verify the passed port number.
//                 SioWaitFor verifies that the baud rate > 0
//  5.3.0 11/04/13 SioSetInteger no longer requires an open port for global (all ports) parameters
//                 Modified SioReset to make it more tolerant opening slow virtual ports.
//  5.3.1  2/04/14 INTERNAL: Added debug stuff to TrialRun.inc
//                 INTERNAL: Removed call to trialSetKeyCode() from SioReset
//  5.3.2  2/13/14 SioRead() removed.
//  5.3.3  6/10/15 Changed "LANG_SYSTEM_DEFAULT" to "MAKELANGID(LANG_NEUTRAL,SUBLANG_DEFAULT)" in SioWinError()
//  5.4.0  7/29/15 Added SioCRC16(), computes 16-bit CCITT CRC using polynomial 0x1021
//                 Added SioCRC32(), computes 32-bit CCITT CRC using polynomial 0x04C11DB7
//         7/31/15 Added SioCountWait() that waits for specified number of incoming bytes before returning.
//                 INTERNAL: oioInit() pointer/integer conversion error fixed.
//  5.4.1  8/17/15 Default for overlapped I/O is off (since large number of USB-RS232 converters)
//                 INTERNAL: Removed SioCountWait (use SioRxWait instead).
//  6.0.0  3/13/17 Added error codes : WSC_BUFFER_RANGE,WSC_BUFLEN_RANGE,WSC_BAD_CMD,WSC_BAD_PARITY.WSC_BAD_STOPBIT,WSC_BAD_WORDLEN
//                 Added SioErrorText() : returns text associated with specified error code
//                 Added SioPortInfo() : returns baud in bps & (theoretical) port CPS
//                 Added SioInfo('O') : returns TRUE if overlapped I/O is enabled, else returns FALSE
//                 Added SioGetsC() : receives an entire line through the stop character (usually CR)
//
// ****** IN TEST ************
///#define DEBUG_THREAD_STATUS
// ***************************
//
//---------------------------------------------------------
//
//*** Visual Studio 2005: Uncomment the following 2 lines
//#pragma once
//#define _CRT_SECURE_NO_DEPRECATE 1
//***
//---------------------------------------------------------

#include "stdafx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <assert.h>

#define NBR_PORTS  256

#define DLL_SOURCE_CODE
#define WIN32_LEAN_AND_MEAN

#ifndef STRICT
#define STRICT
#endif

//#ifndef WIN32
//#define WIN32
//#endif

//#define MIN(a,b) ((a<=b)?(a):(b))
//#define MAX(a,b) ((a>=b)?(a):(b))

#pragma hdrstop

//---------------------------------------------------------

#include "wsc64.h"
#include "timer.h"
#include "trace.h"
#include "enumser.h"
#include "keycode.h"

#include "build.h"

#ifndef USHORT
  #define USHORT unsigned short
#endif

#ifndef UINT
  #define UINT unsigned int
#endif

#ifndef RESETDEV
  #define RESETDEV  7
#endif

#ifdef LCC_CONSTANTS
  #ifndef MAXDWORD
    #define MAXDWORD  0xffffffff
  #endif
  #ifndef LANG_SYSTEM_DEFAULT
    #define LANG_SYSTEM_DEFAULT 2048
  #endif
#endif

#define IHV INVALID_HANDLE_VALUE

#define DtrMask 0x01
#define RtsMask 0x02

// Microsoft: _MSC_VER
// Borland:   __BORLANDC__
// Watcom:    __WATCOMC__
// MinGW:     __GNUC__

#ifdef __GNUC__
// MinGW gcc
#endif

#ifdef _WIN64
#else
#endif

#define RESET_ATTEMPTS 20
#define RESET_SLEEP    50

//---------------------------------------------------------


//---------------------------------------------------------

static volatile int Initialized = 0;
static char *VerAndBld = "$VER 0600 BLD 0006 MAR17 WSC$";  // <-- values are hex.

static COMMTIMEOUTS NewTimeouts;

// default timeout values
static DWORD ReadIntervalTimeout = MAXDWORD;
static DWORD ReadTotalTimeoutMultiplier = 0;
static DWORD ReadTotalTimeoutConstant = 0;
static DWORD WriteTotalTimeoutMultiplier = 0;
static DWORD WriteTotalTimeoutConstant = 0;

static DCB PortDCB[NBR_PORTS];    // initialized by VerifyPort()

static struct
{volatile HANDLE ComHandle;
 volatile HANDLE MsgHandle;
 //volatile HANDLE EventSignal;
 //volatile HANDLE EventMutex;      // acquire within Sio functions only
 //volatile HANDLE WaitCommSignal;  // used to kill background event initialed by WaitCommEvent
 volatile BYTE   WaitCommFlag;      // set TRUE if WaitCommEvent() was cancelled (by setting WaitCommSignal)
 volatile BYTE   Status;
 volatile BYTE   NextChar;
#ifdef DEBUG_THREAD_STATUS
 volatile BYTE   EventThreadStatus;
#endif
 DCB    *DCBptr;
 COMMTIMEOUTS OldTimeouts;
 COMMTIMEOUTS Timeouts;
 HWND   hMsgWnd;
 DWORD  Mask;
 UINT   Port;
 WORD   MsgCode;
 int    PutStatus;
 BYTE   DataMask;
 BYTE   NoWaitFlag;
 int    WaitComm;
 DWORD  ReadPendingTimeout;
} PortData[NBR_PORTS];

static COMSTAT ComStat;
static char ComText[] = "\\\\.\\COM1\0\0\0";


static unsigned BaudValue[10] =
  {110,300,1200,2400,4800,9600,19200,38400,57600,115200};

static unsigned BaudRateDefault = 19200;
static BYTE ParityDefault = WSC_NoParity;
static BYTE StopBitsDefault = WSC_OneStopBit;
static BYTE DataBitsDefault = WSC_WordLength8;
static BYTE DTR_Default = DTR_CONTROL_DISABLE;
static BYTE RTS_Default = RTS_CONTROL_DISABLE;
static BYTE ResetDeviceFlag = FALSE;
static BYTE IgnoreSomeReturnCodes = TRUE;
static volatile int EnterMsgThreadCount = 0;
static volatile int ExitMsgThreadCount = 0;

static volatile int DebugInteger1 = 0; // used for debugging
static volatile int DebugInteger2 = 0; // used for debugging
static volatile int CannotCloseHandleCount = 0;
static volatile int EnableMsgBox = 1;


//                                     1         2         3
//                             0123456789012345678901234567890..
static char *Registration = "$OEM 00000000 0000 X-OEM: OEM Version.                                      \0";
///static int DaysLeft = 999;

///static int  DebugValue;

static DWORD LastWinError = 0;
static int   IsWindows95  = 0;  // TRUE if Windows 95

// CRC Notes
//
// see https://en.wikipedia.org/wiki/Cyclic_redundancy_check
//
// Test values verified with:
//  (1) http://www.simplycalc.com/crc32-text.php
//  (2) http://www.zorc.breitbandkatze.de/crc.html
//

static USHORT CRC16Table[256] = {
   0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
   0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
   0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
   0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
   0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
   0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
   0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
   0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
   0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
   0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
   0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
   0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
   0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
   0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
   0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
   0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
   0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
   0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
   0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
   0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
   0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
   0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
   0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
   0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
   0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
   0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
   0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
   0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
   0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
   0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
   0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
   0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

static unsigned int CRC32Table[256] = {
   0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
   0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
   0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
   0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
   0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
   0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
   0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
   0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
   0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
   0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
   0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
   0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
   0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
   0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
   0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
   0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
   0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
   0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
   0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
   0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
   0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
   0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
   0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
   0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
   0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
   0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
   0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
   0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
   0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
   0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
   0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
   0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d};

//---------------------------------------------------------


static struct
{int   Code;
 char *Text;
} ErrorList[] =
{
 {WSC_ABORTED,      "Aborted"},
 {WSC_BAD_CMD,      "No such command"},
 {WSC_BAD_PARITY,   "Bad parity parameters"},
 {WSC_BAD_STOPBIT,  "Bad stop bit parameter"},
 {WSC_BAD_WORDLEN,  "Bad word length parameter"},
 {WSC_BUFFER_RANGE, "Parameter (buffer address) out of range "},
 {WSC_BUFFERS,      "Cannot allocate memory for buffers"},
 {WSC_BUFLEN_RANGE, "Parameter (buffer length) out of range "},
 {WSC_BUSY,         "Port is busy (try again later)"},
 {WSC_EXPIRED,      "Evaluation version expired, or SioKeyCode not called"},
 {WSC_IE_BADID,     "Invalid COM port"},
 {WSC_IE_BAUDRATE,  "Unsupported baud rate"},
 {WSC_IE_BYTESIZE,  "Unsupported byte size"},
 {WSC_IE_DEFAULT,   "Error in default parameters"},
 {WSC_IE_HARDWARE,  "COM port hardware not present"},
 {WSC_IE_MEMORY,    "Cannot allocate memory"},
 {WSC_IE_NOPEN,     "Cannot open COM port"},
 {WSC_IE_OPEN,      "COM port already open"},
 {WSC_IO_ERROR,     "Event I/O error (virtual port not ready?)"},
 {WSC_KEYCODE,      "Bad key code."},
 {WSC_NO_DATA,      "No (incoming) data"},
 {WSC_RANGE,        "Parameter out of range "},
 {WSC_THREAD,       "Cannot start thread"},
 {WSC_TIMEOUT,      "Operation timed out"},
 {999,NULL}
};


#ifndef STATIC_LIBRARY

BOOL WINAPI DllEntryPoint( HINSTANCE hinstDll,
   DWORD fdwRreason,
   LPVOID plvReserved)
{
 return 1;
}

int FAR PASCAL WEP ( int bSystemExit )
{
 return 1;
}

#endif

//---------------------------------------------------------

//*** PRIVATE functions ***

#ifndef _MSGBOX_
static void MsgBox(char *Ptr)
{MessageBox(NULL,Ptr,(char *)"Info",MB_TASKMODAL|MB_ICONEXCLAMATION);
}
#endif

//---------------------------------------------------------

static int IsMachineWin95(void)
{DWORD WindowsVersion;
 DWORD MajorVersion;
 DWORD MinorVersion;
 WindowsVersion = 0; //GetVersion();
 MajorVersion =  (DWORD) (LOBYTE(LOWORD(WindowsVersion)));
 MinorVersion =  (DWORD) (HIBYTE(LOWORD(WindowsVersion)));
 if((MajorVersion==4)&&(MinorVersion==0)) return TRUE;
 else return FALSE;
}

//-------------[ Overlapped I/O functions ]----------------

#define WSC_IO_READY     0

typedef struct
{HANDLE   hComm;            // serial port handle
 volatile OVERLAPPED OverlappedRead;   // OIO READ data
 volatile OVERLAPPED OverlappedWrite;  // OIO WRITE data
 volatile OVERLAPPED OverlappedEvent;  // OIO EVENT data
 volatile int        LastReadResult;   // last OIO READ result
 volatile int        LastWriteResult;  // last OIO WRITE result
 volatile int        LastEventResult;  // last OIO EVENT result
 volatile DWORD      BytesRead;
 volatile DWORD      BytesWritten;
} oioStuffType;

static oioStuffType oioStuff[NBR_PORTS];  // index by port (COM1, COM2, ...)
static int oioEnableOverlappedIO = FALSE; // default: overlapped I/O is disabled

//***********************************************************************
//*** NOTE: Pass NULL for oioPtr to do non-overlapped serial port I/O ***
//***       Win95 does NOT support (serial port) overlapped I/O       ***
//***       Win98 thru WinXP supports (serial port) overlapped I/O    ***
//***********************************************************************

//--- __oioCreateEvent() called from oioInit() ONLY ---

static HANDLE __oioCreateEvent(LPOVERLAPPED oioPtr)
{// create MANUAL RESET event for overlapped I/O.
 oioPtr->hEvent = CreateEvent(
                        NULL,   // default security attributes
                        TRUE,   // manual reset event
                        FALSE,  // not signaled
                        NULL);  // no name
 // intialize the rest of the OVERLAPPED structure to zero.
 oioPtr->Internal = 0;
 oioPtr->InternalHigh = 0;
 oioPtr->Offset = 0;
 oioPtr->OffsetHigh = 0;
 assert(oioPtr->hEvent);
 return oioPtr->hEvent;
}

//---------------------------------------------------------

// signal event

static int oioEventSignal(int Port)
{LPOVERLAPPED oioPtr;
 if(!oioEnableOverlappedIO) return 1;
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedEvent;
 if(oioPtr) SetEvent(oioPtr->hEvent);
 return 1;
}

//---------------------------------------------------------

// initial oio for one port

static int oioInit(int Port, HANDLE hComm)
{HANDLE Handle;
 LPOVERLAPPED oioPtr;
 // initialize counts
 oioStuff[Port].BytesRead = 0;
 oioStuff[Port].BytesWritten = 0;
 // save hComm
 oioStuff[Port].hComm = hComm;
 if(!oioEnableOverlappedIO)
   {// no overlapped I/O
    return WSC_IO_COMPLETE;
   }
 // create READ signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedRead;
 Handle = __oioCreateEvent(oioPtr);
 if(Handle==NULL) return WSC_IO_ERROR;
 // create WRITE signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedWrite;
 Handle = __oioCreateEvent(oioPtr);
 if(Handle==NULL) return WSC_IO_ERROR;
 // create EVENT signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedEvent;
 Handle = __oioCreateEvent(oioPtr);
 if(Handle==NULL) return WSC_IO_ERROR;
 // initial "last result"
 oioStuff[Port].LastReadResult  = WSC_IO_READY;
 oioStuff[Port].LastWriteResult = WSC_IO_READY;
 oioStuff[Port].LastEventResult = WSC_IO_READY;
 return WSC_IO_COMPLETE;
}

//---------------------------------------------------------

static int oioDone(int Port)
{LPOVERLAPPED oioPtr;
 if(!oioEnableOverlappedIO) return WSC_IO_COMPLETE;
 // close Overlappped READ signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedRead;
 CloseHandle(oioPtr->hEvent);
 oioPtr->hEvent = 0;
 // close Overlappped WRITE signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedWrite;
 CloseHandle(oioPtr->hEvent);
 oioPtr->hEvent = 0;
 // close Overlappped EVENT signal
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedEvent;
 CloseHandle(oioPtr->hEvent);
 oioPtr->hEvent = 0;
 return WSC_IO_COMPLETE;
}

//---------------------------------------------------------

// start overlapped event

static int oioEventStart(int Port, DWORD *MaskPtr)
{LPOVERLAPPED oioPtr;
 HANDLE hComm;
 if(!oioEnableOverlappedIO) return WSC_IO_COMPLETE;
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedEvent;
 hComm = oioStuff[Port].hComm;
 // set mask
 if(!SetCommMask(hComm, *MaskPtr))
   {oioStuff[Port].LastEventResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
 if(oioPtr!=NULL) ResetEvent(oioPtr->hEvent);
 // setup wait comm event
 if(WaitCommEvent(hComm, MaskPtr, oioPtr))
   {// event has ocurred
    oioStuff[Port].LastEventResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 else
   {// error or event is pending
    DWORD LastWinError = GetLastError();
    if(LastWinError==ERROR_IO_PENDING)
      {oioStuff[Port].LastEventResult = WSC_IO_PENDING;
       ///MsgBox((char *)"WSC_IO_PENDING"); ////////////////
       return WSC_IO_PENDING;
      }
    else
      {oioStuff[Port].LastEventResult = WSC_IO_ERROR;
       ///MsgBox((char *)"WSC_IO_ERROR"); ////////////////
       return WSC_IO_ERROR;
      }
   }
}

//---------------------------------------------------------

// start overlapped read
// -- returns with whatever data is available; does not wait for 'BufSize' bytes

static int oioReadStart(int Port, char *Buffer, DWORD BufSize)
{int Code;
 int WinError;
 ///DWORD Errors;
 ///COMSTAT ComStat;
 LPOVERLAPPED oioPtr;
 HANDLE hComm;
 if(!oioEnableOverlappedIO) oioPtr = NULL;
 else
   {oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedRead;
    if(oioStuff[Port].LastReadResult==WSC_IO_PENDING) return WSC_BUSY;
   }
 hComm = oioStuff[Port].hComm;
 ///ClearCommError(hComm, &Errors, &ComStat);
 *Buffer = '\0';
 oioStuff[Port].BytesRead = 0;
 Code = ReadFile((HANDLE)hComm,
                 (LPVOID)Buffer,
                 (DWORD)BufSize,
                 (LPDWORD)&oioStuff[Port].BytesRead,
                 (LPOVERLAPPED)oioPtr);
 if(Code)
   {oioStuff[Port].LastReadResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 // ReadFile failure ?
 WinError = GetLastError();
 if(WinError==ERROR_IO_PENDING)
   {///oioStuff[Port].BytesRead = 0;
    oioStuff[Port].LastReadResult = WSC_IO_PENDING;
    return WSC_IO_PENDING;
   }
 else
   {oioStuff[Port].LastReadResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
}

//---------------------------------------------------------

// start overlapped write

static int oioWriteStart(int Port, char *Buffer, DWORD BufSize)
{int Code;
 ///DWORD Errors;
 ///COMSTAT ComStat;
 int WinError;
 LPOVERLAPPED oioPtr;
 HANDLE hComm;
 if(!oioEnableOverlappedIO) oioPtr = NULL;
 else
   {if(oioStuff[Port].LastWriteResult==WSC_IO_PENDING) return WSC_BUSY;
    oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedWrite;
   }

 hComm = oioStuff[Port].hComm;
 ///ClearCommError(hComm, &Errors, &ComStat);
 oioStuff[Port].BytesWritten = 0;
 Code = WriteFile((HANDLE)hComm,
                  (LPVOID)Buffer,
                  (DWORD)BufSize,
                  (LPDWORD)&oioStuff[Port].BytesWritten,
                  (LPOVERLAPPED)oioPtr);
 if(Code)
   {oioStuff[Port].LastWriteResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 // ReadFile failure ?
 WinError = GetLastError();
 if(WinError==ERROR_IO_PENDING)
   {///oioStuff[Port].BytesWritten = 0;
    oioStuff[Port].LastWriteResult = WSC_IO_PENDING;
    return WSC_IO_PENDING;
   }
 else
   {oioStuff[Port].LastWriteResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
}

//---------------------------------------------------------

// check overlapped READ status

static int oioReadWait(int Port, DWORD Timeout)
{int Code;
 LPOVERLAPPED oioPtr;
 if(!oioEnableOverlappedIO) return WSC_IO_COMPLETE;
 if(oioStuff[Port].LastReadResult == WSC_IO_COMPLETE) return WSC_IO_COMPLETE;
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedRead;
 if(oioPtr==NULL) return WSC_IO_COMPLETE;
 Code = WaitForSingleObject(oioPtr->hEvent, Timeout);
 if(Code==WAIT_OBJECT_0)
   {oioStuff[Port].LastReadResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 if(Code==WAIT_TIMEOUT)
   {oioStuff[Port].LastReadResult = WSC_IO_PENDING;
    return WSC_IO_PENDING;
   }
 else
   {oioStuff[Port].LastReadResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
}

//---------------------------------------------------------

// check overlapped WRITE status

static int oioWriteWait(int Port, DWORD Timeout)
{int Code;
 LPOVERLAPPED oioPtr;
 if(!oioEnableOverlappedIO) return WSC_IO_COMPLETE;
 if(oioStuff[Port].LastWriteResult == WSC_IO_COMPLETE) return WSC_IO_COMPLETE;
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedWrite;
 if(oioPtr==NULL) return WSC_IO_COMPLETE;
 Code = WaitForSingleObject(oioPtr->hEvent, Timeout);
 if(Code==WAIT_OBJECT_0)
   {oioStuff[Port].LastWriteResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 if(Code==WAIT_TIMEOUT)
   {oioStuff[Port].LastWriteResult = WSC_IO_PENDING;
    return WSC_IO_PENDING;
   }
 else
   {oioStuff[Port].LastWriteResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
}

//---------------------------------------------------------

// check overlapped EVENT status

static int oioEventWait(int Port, DWORD Timeout)
{int Code;
 LPOVERLAPPED oioPtr;
 if(!oioEnableOverlappedIO) return WSC_IO_COMPLETE;
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedEvent;
 if(oioPtr==NULL) return WSC_IO_COMPLETE;
 Code = WaitForSingleObject(oioPtr->hEvent, Timeout);
 if(Code==WAIT_OBJECT_0)
   {oioStuff[Port].LastEventResult = WSC_IO_COMPLETE;
    return WSC_IO_COMPLETE;
   }
 if(Code==WAIT_TIMEOUT)
   {oioStuff[Port].LastEventResult = WSC_IO_PENDING;
    return WSC_IO_PENDING;
   }
 else
   {oioStuff[Port].LastEventResult = WSC_IO_ERROR;
    return WSC_IO_ERROR;
   }
}

//---------------------------------------------------------

#if 0
static int oioReadStatus(int Port)
{
 return oioStuff[Port].LastReadResult;
}
#endif

//---------------------------------------------------------

// get last WRITE result

static int oioWriteStatus(int Port)
{
 return oioStuff[Port].LastWriteResult;
}

//---------------------------------------------------------

// get bytes READ

DWORD oioGetBytesRead(int Port)
{DWORD BytesTransferred;
 LPOVERLAPPED oioPtr;
 HANDLE hComm;
 if(!oioEnableOverlappedIO) return oioStuff[Port].BytesRead;
 if(oioStuff[Port].LastReadResult==WSC_IO_COMPLETE)
   {
    if(oioStuff[Port].BytesRead>0) return oioStuff[Port].BytesRead;
   }
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedRead;
 hComm = oioStuff[Port].hComm;
 // NOTE: call GetOverlappedResult ONLY after getting WSC_IO_PENDING
 if(GetOverlappedResult(hComm, oioPtr, (LPDWORD)&BytesTransferred, TRUE))
   {if(BytesTransferred>(DWORD)0)
      {oioStuff[Port].BytesRead = BytesTransferred;
       return BytesTransferred;
      }
   }
 return 0;
}

//---------------------------------------------------------

// get bytes WRITTEN

DWORD oioGetBytesWritten(int Port)
{DWORD BytesTransferred;
 LPOVERLAPPED oioPtr;
 HANDLE hComm;
 if(!oioEnableOverlappedIO) return oioStuff[Port].BytesRead;
 if(oioStuff[Port].LastWriteResult==WSC_IO_COMPLETE)
   {
    if(oioStuff[Port].BytesWritten>0) return oioStuff[Port].BytesWritten;
   }
 oioPtr = (LPOVERLAPPED)&oioStuff[Port].OverlappedWrite;
 hComm = oioStuff[Port].hComm;
 if(GetOverlappedResult(hComm, oioPtr, (LPDWORD)&BytesTransferred, TRUE))
   {if(BytesTransferred>(DWORD)0)
      {oioStuff[Port].BytesWritten = BytesTransferred;
       return BytesTransferred;
      }
   }
 return 0;
}

//---------------------------------------------------------

static int VerifyPort(int Port)
{
 if((Port<0)||(Port>=NBR_PORTS)) return WSC_IE_BADID;
 else return 0;
}

//---------------------------------------------------------

int CloseTheHandle(HANDLE Handle)
{int Code;
 Code = CloseHandle(Handle);
 if(!Code) CannotCloseHandleCount++;
 return Code;
}

//---------------------------------------------------------

//// Thread functions **

void MsgThread(PVOID pvoid)
{int Port;
 HWND  hMsgWnd;
 DWORD Mask;
 WORD  MsgCode;
 int   *PortPtr;
 PortPtr = (int *) pvoid;
 Port = *PortPtr;
 hMsgWnd = PortData[Port].hMsgWnd;
 Mask = PortData[Port].Mask;
 MsgCode = PortData[Port].MsgCode;
 EnterMsgThreadCount++;
 // SioEvent will block this thread until the event occurs
 SioEvent(Port, Mask);
 // event has occured: send message
 if(hMsgWnd) PostMessage(hMsgWnd, MsgCode, (WORD)Port, 0L);
 if(PortData[Port].MsgHandle)
   {// close thread handle
    if(CloseTheHandle(PortData[Port].MsgHandle)) PortData[Port].MsgHandle = 0;
   }
 // exit thread
 ExitMsgThreadCount++;
 ExitThread(0);
}

//---------------------------------------------------------

void EventThreadError(char *Message, int ErrCode)
{char xxx[128];
 if(EnableMsgBox)
   {wsprintf((char *)xxx,"Error %d : %s", ErrCode, Message);
    MsgBox((char *)xxx);
   }
}

//---------------------------------------------------------

// terminate thread if running

static int TerminateTheThread(HANDLE Handle)
{int Code;
 DWORD ThreadStatus;
 // only Win95 code needs to terminate a running thread !
 if(!IsWindows95) return FALSE;
 // get the thread status
 GetExitCodeThread(Handle, &ThreadStatus);
 // is thread still running ?
 if(ThreadStatus==STILL_ACTIVE)
   {// attempt to terminate thread
    Code = TerminateThread(Handle, 0);
    if(!Code)
      {if(EnableMsgBox)
         {char xxx[64];
          wsprintf((char *)xxx,(char *)"Cannot terminate thread %x",Handle);
          MsgBox((char *)xxx);
         }
      }
    return TRUE;
   }
 return FALSE;
}

//---------------------------------------------------------

static void InitializeIfNeeded(void)
{int i;
 // have we already initialized ?
 if(Initialized==0)
   {Initialized = 1;
    // is this Windows 95 ?
    IsWindows95 = IsMachineWin95();

    for(i=0;i<NBR_PORTS;i++)
      {PortData[i].ComHandle = IHV;
       PortData[i].Status = 0;
       PortData[i].NextChar = '\0';
       PortData[i].DCBptr = &PortDCB[i];
      }
    }
}

//---------------------------------------------------------
//
// PUBLIC functions

NoMangle int DLL_IMPORT_EXPORT SioKeyCode(unsigned int KeyCode)
{

 InitializeIfNeeded();




 return 999;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioGetReg(char *Buffer, int BufLen)
{int Len = 0;
 InitializeIfNeeded();
 return Len;
}

//---------------------------------------------------------

static int SetTimeouts(HANDLE hComm)
{// set new timeouts
 NewTimeouts.ReadIntervalTimeout = ReadIntervalTimeout;
 NewTimeouts.ReadTotalTimeoutMultiplier = ReadTotalTimeoutMultiplier;
 NewTimeouts.ReadTotalTimeoutConstant = ReadTotalTimeoutConstant;
 NewTimeouts.WriteTotalTimeoutMultiplier = WriteTotalTimeoutMultiplier;
 NewTimeouts.WriteTotalTimeoutConstant = WriteTotalTimeoutConstant;
 if(!SetCommTimeouts(hComm, &NewTimeouts))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 else return 1;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioReset(int Port, int InQueue, int OutQueue)
{int i;
 HANDLE hComm;
 DCB  *DCBptr;
 int  Code;
 DWORD Errors;
 if(Port==-1)
   {// set initialization defaults for DTR & RTS
    DTR_Default = 0x01 & (BYTE) InQueue;
    RTS_Default = 0x01 & (BYTE) OutQueue;
    return 0;
   }
 if((Code=VerifyPort(Port))<0) return Code;
 InitializeIfNeeded();
 wsprintf((char *)ComText,"\\\\.\\COM%d",Port+1);

 if(oioEnableOverlappedIO)
   hComm = CreateFile((char *)ComText,
                      GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING,
                      FILE_FLAG_NO_BUFFERING|FILE_FLAG_OVERLAPPED,      // enable overlapped I/O
                      0);
 else
    hComm = CreateFile((char *)ComText,
                       GENERIC_READ|GENERIC_WRITE, 0, 0, OPEN_EXISTING,
                       FILE_FLAG_NO_BUFFERING,                           // enable non-overlapped I/O
                      0);

 if(hComm==INVALID_HANDLE_VALUE)
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }

 PortData[Port].ComHandle = hComm;
 oioInit(Port, hComm);

 // get old comm timeouts
 if(!GetCommTimeouts(hComm,&(PortData[Port].OldTimeouts)))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 // get comm state
 DCBptr = PortData[Port].DCBptr;
 DCBptr->DCBlength = sizeof(struct _DCB);
 if(!GetCommState(hComm,DCBptr))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 // set our defaults
 DCBptr->BaudRate = BaudRateDefault;
 DCBptr->ByteSize = DataBitsDefault;
 DCBptr->Parity   = ParityDefault;
 DCBptr->StopBits = StopBitsDefault;
 DCBptr->fBinary = 1;

 DCBptr->fNull = 0;
 DCBptr->fOutxDsrFlow = 0;
 DCBptr->fOutxCtsFlow = 0;

 if(DTR_Default) DCBptr->fDtrControl = DTR_CONTROL_ENABLE;
 else DCBptr->fDtrControl = DTR_CONTROL_DISABLE;
 if(RTS_Default) DCBptr->fRtsControl = RTS_CONTROL_ENABLE;
 else DCBptr->fRtsControl = RTS_CONTROL_DISABLE;
 DCBptr->fInX = 0;
 DCBptr->fOutX = 0;
 DCBptr->fTXContinueOnXoff = 1;
 DCBptr->fDsrSensitivity = 0;
 DCBptr->fAbortOnError = 0;
 DCBptr->XonChar = 0x11;
 DCBptr->XoffChar = 0x13;
 if(InQueue<=512)
   {DCBptr->XonLim =  (UINT)(InQueue/4);
    DCBptr->XoffLim = (UINT)(InQueue/4);
   }
 else
   {DCBptr->XonLim =  128;
    DCBptr->XoffLim = 128;
   }
 //END$SHAREWARE
 // set the DCB
 if(!SetCommState(hComm,DCBptr))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 // setup I/O buffers
 Code = SetupComm(hComm,InQueue,OutQueue);
 if((!IgnoreSomeReturnCodes)&&(!Code)) return WSC_BUFFERS;
 // reset device ?
 if(ResetDeviceFlag)
   {// RESET the port
    if(!EscapeCommFunction(hComm,RESETDEV))
      {LastWinError = GetLastError();
       return WSC_WIN32ERR;
      }
   }

 ClearCommError(hComm,&Errors,&ComStat);
 // set comm event mask
 //if(!SetCommMask(hComm,EV_RXCHAR|EV_BREAK))
 //  {LastWinError = GetLastError();
 //   return WSC_WIN32ERR;
 //  }

 // set new timeouts
 Code = SetTimeouts(hComm);
 if(Code<0) return Code;
 // port is open
 PortData[Port].Port = Port;
 PortData[Port].ComHandle = hComm;
 PortData[Port].DataMask = (1 << DataBitsDefault) - 1;
 PortData[Port].PutStatus = 0;
 PortData[Port].ReadPendingTimeout = 50;
 if(!oioEnableOverlappedIO) return 0;
 // make sure we can read from port
 Sleep(1);
 PortData[Port].ReadPendingTimeout = 1000;
 for(i=1;i<=RESET_ATTEMPTS;i++)
   {Code = SioGetc(Port);
    if(Code==WSC_NO_DATA) return 1;
    if(Code>=0)
      {// save incoming byte
       PortData[Port].NextChar = (char) Code;
       return 2;
      }
    // allow more time
    Sleep(RESET_SLEEP);
   }
 // unloaded virtual COM port ?
 return WSC_IO_ERROR;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioDone(int Port)
{HANDLE hComm;
 int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 // unblock SioEvent (which will also allow MsgThread to unblock)
 oioEventSignal(Port);
 if(PortData[Port].MsgHandle)
   {// terminate thread if running
    TerminateTheThread(PortData[Port].MsgHandle);
    PortData[Port].MsgHandle = 0;
   }
 // restore old timeouts
 SetCommTimeouts(hComm,&(PortData[Port].OldTimeouts));
 PortData[Port].ComHandle = IHV;
 oioDone(Port);
 return CloseTheHandle(hComm);
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioBaud(int Port, unsigned int BaudRate)
{HANDLE hComm;
 int Code;
 // is BaudRate an index ?
 if(BaudRate<=WSC_Baud115200) BaudRate = BaudValue[BaudRate];
 if(Port==-1)
   {BaudRateDefault = BaudRate;
    return 0;
   }
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 PortDCB[Port].BaudRate = BaudRate;
 return SetCommState(hComm,&PortDCB[Port]);
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioPutc(int Port, char Byte)
{HANDLE hComm;
 int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;

 // was there a previous WRITE ?

 Code = oioWriteStatus(Port);
 if(Code==WSC_IO_PENDING)
    {// yes, WRITE was previously started
     Code = oioWriteWait(Port, 0);
     if(Code==WSC_IO_PENDING)  return WSC_BUSY;
    }

 // start the write
 Code = oioWriteStart(Port,(char *)&Byte, 1);
 if(Code==WSC_IO_ERROR) return Code; ////

 // wait for write to complete
 Code = oioWriteWait(Port, INFINITE);
 if(Code==WSC_IO_COMPLETE)
   {// return byte count
    return oioGetBytesWritten(Port);
   }
 else return 0;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioPuts(int Port, char *Buffer, unsigned Size)
{int Code;
 HANDLE hComm;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 if(Buffer==NULL) return WSC_BUFFER_RANGE;
 if(Size==0) return WSC_BUFLEN_RANGE;

 ///if(Size==0) if(Code==WSC_IO_PENDING)  return WSC_BUSY;

 // was there a previous WRITE ?

 Code = oioWriteStatus(Port);
 if(Code==WSC_IO_PENDING)
    {// yes, WRITE was previously started
     Code = oioWriteWait(Port, 0);
     if(Code==WSC_IO_PENDING)  return WSC_BUSY;
    }


 // start the write
 Code = oioWriteStart(Port, Buffer, Size);
    ///if(Code==WSC_BUSY) return WSC_BUSY;
 if(Code==WSC_IO_ERROR) return Code;

 if(PortData[Port].NoWaitFlag)
   {// return immediately
    return 1;
   }
 else
   {// wait for SioPuts to complete before returning
    Code = oioWriteWait(Port, INFINITE);
    if(Code==WSC_IO_COMPLETE)
       {// return byte count
        return oioGetBytesWritten(Port);
       }
    else return 0;
   }
 //return 0;
}

//---------------------------------------------------------
//
// Get incoming data until the specified stop char is received or timeout occurs,
// returning the number of bytes read
//
// A full "line" was read if the last character in Buffer[] was the StopChr

NoMangle int DLL_IMPORT_EXPORT SioGetsC(int Port, char *Buffer, int BufLen, unsigned int Timeout, char StopChr)
{int CPS;
 int Bytes = 0;
 int Code;
 unsigned int TimeMark;
 unsigned int mSecPerChar;
 if((Code=VerifyPort(Port))<0) return Code;
 CPS = SioPortInfo(Port, 'C');
 if(CPS<0) return CPS;
 mSecPerChar = 1000 / CPS;
 if(mSecPerChar<1) mSecPerChar = 1;
 // compute max time to wauit for line
 TimeMark = SioTimer() + Timeout;
 while(SioTimer() <= TimeMark)
   {Code = SioGetc(Port);
    if(Code==WSC_NO_DATA) Sleep(mSecPerChar);
    else
      {if(Code<0) return Code;
       // got data
       *Buffer++ = (char)Code;
       Bytes++;
       if(Bytes==BufLen) return Bytes;
       if((char)Code==StopChr) return Bytes;
      }
   }
 return Bytes;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioGets(int Port,char *Buffer,unsigned Size)
{HANDLE hComm;
 DWORD Errors;
 char  Byte;
 int   Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 Byte = PortData[Port].NextChar;
 if(Byte!='\0')
   {PortData[Port].NextChar = '\0';
    return 0x00ff & (int)Byte;
   }
 ClearCommError(hComm,&Errors,&ComStat);
 // start READ
 Code = oioReadStart(Port,Buffer, Size);
 if(Code == WSC_IO_ERROR) return Code;
 // wait for READ to complete
 oioReadWait(Port, INFINITE);
 return oioGetBytesRead(Port);
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioGetc(int Port)
{HANDLE hComm;
 DWORD Errors;
 long  BytesRead;
 char  Byte;
 int   Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 Byte = PortData[Port].NextChar;
 if(Byte!='\0')
   {PortData[Port].NextChar = '\0';
    return 0x00ff & (int)Byte;
   }
 ClearCommError(hComm,&Errors,&ComStat);
 // start READ
 Code = oioReadStart(Port,(char *)&Byte, 1);
 if(Code == WSC_IO_ERROR) return Code;

 // wait for READ to complete
    ///oioReadWait(Port, INFINITE);
 Code = oioReadWait(Port, PortData[Port].ReadPendingTimeout);

 // timeout if still pending
 if(Code==WSC_IO_PENDING) return WSC_IO_ERROR;

 BytesRead = oioGetBytesRead(Port);
 if(BytesRead<=0) return WSC_NO_DATA;
 // return byte
 return PortData[Port].DataMask & (int)Byte;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioDTR(int Port,char Cmd)
{HANDLE hComm;
 int  Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 switch(Cmd)
   {case 'S':
      PortData[Port].Status |= DtrMask;
      return EscapeCommFunction(hComm,SETDTR);
    case 'C':
      PortData[Port].Status &= (~DtrMask);
      return EscapeCommFunction(hComm,CLRDTR);
    case 'R':
      return DtrMask & PortData[Port].Status;
    default:
      return WSC_BAD_CMD;
   }
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioRTS(int Port,char Cmd)
{HANDLE hComm;
 int  Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 switch(Cmd)
   {case 'S':
      PortData[Port].Status |= RtsMask;
      return EscapeCommFunction(hComm,SETRTS);
    case 'C':
      PortData[Port].Status &= (~RtsMask);
      return EscapeCommFunction(hComm,CLRRTS);
    case 'R':
      return RtsMask & PortData[Port].Status;
    default:
      return WSC_BAD_CMD;
   }
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioTxClear(int Port)
{HANDLE hComm;
 int  Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 PurgeComm(hComm, PURGE_TXCLEAR);
 return 0;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioRxClear(int Port)
{HANDLE hComm;
 int  Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 PurgeComm(hComm, PURGE_RXCLEAR);
 PortData[Port].NextChar = '\0';
 return 0;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioTxQue(int Port)
{HANDLE hComm;
 DWORD Errors;
 int   Code;
 int   TxBytes;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 if(ClearCommError(hComm,&Errors,&ComStat)) TxBytes = ComStat.cbOutQue;
 else TxBytes = 0;

 if(TxBytes<0) TxBytes = 0;

 if(PortData[Port].NoWaitFlag) return TxBytes + (int) oioStuff[Port].BytesWritten;
 else return TxBytes;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioRxQue(int Port)
{HANDLE hComm;
 DWORD Errors;
 int  Code;
 int RxBytes;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 if(ClearCommError(hComm,&Errors,&ComStat)) RxBytes = ComStat.cbInQue;
 else RxBytes = 0;
 return RxBytes;
}

//---------------------------------------------------------
//
//   ...Mask...   :  ...Description...
//   WSC_RXOVER   :  The receive queue overflowed.
//   WSC_OVERRUN  :  An incoming byte was overwritten.
//   WSC_PARITY   :  A parity error was detected (incoming byte)
//   WSC_FRAME    :  A framing error was detected (incoming byte)
//   WSC_BREAK    :  A break signal was detected.
//   WSC_TXFULL   :  The transmit queue is full.
//

NoMangle int DLL_IMPORT_EXPORT SioStatus(int Port, unsigned Mask)
{HANDLE hComm;
 DWORD Errors = 0;
 int   Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 if(!ClearCommError(hComm,&Errors,&ComStat)) return -1;
 return Mask & Errors;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioFlow(int Port,char Cmd)
{HANDLE hComm;
 int  Code;
 DCB  *DCBptr;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 DCBptr = PortData[Port].DCBptr;
 switch(Cmd)
   {case 'H':  // HW flow control
      DCBptr->fOutxCtsFlow = 1;
      DCBptr->fRtsControl = RTS_CONTROL_HANDSHAKE;
      DCBptr->fInX = 0;
      DCBptr->fOutX = 0;
      break;
    case 'S': // SW (XON/XOFF) flow control
      DCBptr->fOutxCtsFlow = 0;
      DCBptr->fRtsControl = RTS_CONTROL_DISABLE;
      DCBptr->fInX = 1;
      DCBptr->fOutX = 1;
      break;
    case 'N':  // no flow control
      DCBptr->fOutxCtsFlow = 0;
      DCBptr->fRtsControl = RTS_CONTROL_DISABLE;
      DCBptr->fInX = 0;
      DCBptr->fOutX = 0;
      break;
    default:
      return WSC_BAD_CMD;
   }
 // set DCB
 if(SetCommState(hComm,DCBptr))
   {// maintain state of DTR & RTS
    if(DtrMask&PortData[Port].Status) EscapeCommFunction(hComm,SETDTR);
    if(RtsMask&PortData[Port].Status) EscapeCommFunction(hComm,SETRTS);
   }
 return 1;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioParms(int Port,int Parity,int StopBits,int DataBits)
{HANDLE hComm;
 int Code;
 DCB *DCBptr;
 if((Parity<WSC_NoParity)||(Parity>WSC_SpaceParity)) return WSC_BAD_PARITY;
 if((StopBits<WSC_OneStopBit)||(StopBits>WSC_TwoStopBits)) return WSC_BAD_STOPBIT;
 if((DataBits<WSC_WordLength5)||(DataBits>WSC_WordLength8)) return WSC_BAD_WORDLEN;
 if(Port==-1)
   {// set defaults
    ParityDefault = (BYTE)Parity;
    StopBitsDefault = (BYTE)StopBits;
    DataBitsDefault = (BYTE)DataBits;
    return 0;
   }
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 DCBptr = PortData[Port].DCBptr;
 // set paramaters
 DCBptr->Parity = Parity;
 DCBptr->StopBits = StopBits;
 DCBptr->ByteSize = DataBits;
 PortData[Port].DataMask = (1 << DataBits) - 1;
 if(SetCommState(hComm,DCBptr))
   {// maintain state of DTR & RTS
    if(DtrMask&PortData[Port].Status) EscapeCommFunction(hComm,SETDTR);
    if(RtsMask&PortData[Port].Status) EscapeCommFunction(hComm,SETRTS);

   }
 return 0;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioCTS(int Port)
{HANDLE hComm;
 int   Code;
 DWORD ModemStatus;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 GetCommModemStatus(hComm,&ModemStatus);
 return MS_CTS_ON & ModemStatus;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioDSR(int Port)
{HANDLE hComm;
 DWORD  ModemStatus;
 int    Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 GetCommModemStatus(hComm,&ModemStatus);
 return MS_DSR_ON & ModemStatus;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioRI(int Port)
{HANDLE hComm;
 DWORD  ModemStatus;
 int    Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 GetCommModemStatus(hComm,&ModemStatus);
 return MS_RING_ON & ModemStatus;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioDebug(int Parm)
{int i;
 switch(Parm)
  {

   case 'R':
     ResetDeviceFlag = TRUE;
     return Parm;
   case 'W':
     //if(NoWaitFlag) NoWaitFlag = FALSE;
     //else NoWaitFlag = TRUE;
     //return Parm;
     for(i=0;i<NBR_PORTS;i++) PortData[i].NoWaitFlag = 1;
     // fall through...
   case 'm':  // toggle (on/off) MsgBox calls (and most calls to wsprintf)
     EnableMsgBox = 1 - EnableMsgBox;
     if(EnableMsgBox) MsgBox((char *)"MsgBox enabled");
     return EnableMsgBox;

  }
 return -1;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioSetInteger(int Port, int ParmName, int ParmValue)
{int Code;

 // does NOT require a port
 switch(ParmName)
  {case 'X':
     // ignore any error returned from SetupComm (problem in some USB/serial implementations)
     IgnoreSomeReturnCodes = (BYTE) ParmValue;
     return (int)ParmValue;
   case 'O':
     // set overlapped mode (for all ports) ?
     if(ParmValue)
        {// try to set overlapped mode
   if(IsMachineWin95())
     {// can't set overlapped mode in Windows 95
      oioEnableOverlappedIO = FALSE;
     }
   else oioEnableOverlappedIO = TRUE;
      }
     else
       {// disable overlapped mode
    oioEnableOverlappedIO = FALSE;
     }
     return oioEnableOverlappedIO;
   case 'M': return IsMachineWin95();
  }

 // requires an (open) port
 if((Code=VerifyPort(Port))<0) return Code;

 switch(ParmName)
  {case 'P':
     PortData[Port].ReadPendingTimeout = ParmValue;
     return (int)ParmValue;
   case 'W':
     PortData[Port].NoWaitFlag = (BYTE)ParmValue;
     return (int)ParmValue;
   case 'H':
     //return (int) PortData[Port].ComHandle;
   case 'S':
     // unblock SioEvent
     oioEventSignal(Port);
     return 2;
   default:
     return -1;
  }
}

//---------------------------------------------------------

#ifndef _READ_HEX_STRING_
static unsigned int ReadHexString(LPCSTR Ptr, int Count)
{int  i, n;
 char c;
 unsigned int Result = 0;
 for(i=0;i<Count;i++)
   {c = *Ptr++;
    n = 0;
    if((c>='0')&&(c<='9')) n = c - '0';
    if((c>='A')&&(c<='F')) n = 10 + c - 'A';
    if((c>='a')&&(c<='f')) n = 10 + c - 'a';
    Result = (Result<<4) + n;
   }
 return Result;
}
#endif


//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioInfo(char Parm)
{InitializeIfNeeded();
 switch(Parm)
   {case 'V':   // version
      return ReadHexString((char *)VerAndBld + 5, 4);
    case 'B':   // build
      return ReadHexString((char *)VerAndBld + 14, 4);

    case 'O':   // overlapped I/O enabled ?
      if(oioEnableOverlappedIO) return TRUE;
      else return FALSE;

#ifdef _X64
    case '3':   // 32 bit ?
      return 0;
    case '6':   // 64 bit ?
      return 1;
#else
    case '3':   // 32 bit ?
      return 1;
    case '6':   // 64 bit ?
      return 0;
#endif

    case 'I':   // TX interrupts always on
      return 1;

    case 'M': return EnterMsgThreadCount;
    case 'm': return ExitMsgThreadCount;

    case 'c': return CannotCloseHandleCount;

    case '1': return DebugInteger1;
    case '2': return DebugInteger2;


    case '?':
      return 999;
    default:
      return -1;
   }
}

//---------------------------------------------------------
// get theoretical port CPS

static double fStopBitsTable[] = {1.0, 1.5, 2.0};
static double fParityTable[]   = {0.0, 1.0, 1.0, 1.0, 1.0};

static double GetTheoryCPS(int Port)
{double fTheoryCPS;
 double fBitsPerByte;
 double fBaudRate;
 double fByteSize;
 double fStopBits;
 double fParity;
 // get raw paramters
 fBaudRate = (double) ((int)PortDCB[Port].BaudRate);
 fByteSize = (double) ((int)PortDCB[Port].ByteSize);
 fStopBits = fStopBitsTable[(int)PortDCB[Port].StopBits];
 fParity = fParityTable[(int) PortDCB[Port].Parity];
 // compute bits per byte
 fBitsPerByte = fByteSize + fStopBits + fParity + 0.66667;
 // compute theoretical maximum CPS
 fTheoryCPS = fBaudRate / fBitsPerByte;
 return (int) (0.5 + fTheoryCPS);
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioPortInfo(int Port, char Parm)
{int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 switch(Parm)
   {case 'B':   // port baud rate in bits/sec
      return (int) PortDCB[Port].BaudRate;
    case 'C':   // port CPS (theoretical, not actual)
      return (int) (0.5 + GetTheoryCPS(Port));
    default:
      return -1;
   }
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioDCD(int Port)
{HANDLE hComm;
 DWORD  ModemStatus;
 int    Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 GetCommModemStatus(hComm,&ModemStatus);
 return MS_RLSD_ON & ModemStatus;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioBrkSig(int Port,char Cmd)
{HANDLE hComm;
 DWORD  Errors;
 int    Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 switch(Cmd)
   {case 'A':  // ASSERT
      return SetCommBreak(hComm);
    case 'C':  // CANCEL
      return ClearCommBreak(hComm);
    case 'D':  // DETECT
      if(!ClearCommError(hComm,&Errors,&ComStat)) return 0;
      return CE_BREAK & Errors;
    default:
      return WSC_BAD_CMD;
   }
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioUnGetc(int Port,char Chr)
{HANDLE hComm;
 int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 PortData[Port].NextChar = Chr;
 return 0;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioWinError(char *Buffer, int Size)
{char LocalBuffer[128];
 char *BufferPtr;
 int BufferSize;
 // use passed buffer ?
 if(Size>0)
   {BufferPtr = Buffer;
    BufferSize = Size;
   }
 else
   {BufferPtr = (char *)&LocalBuffer[0];
    BufferSize = 128;
   }

 // get error message text
 FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM,
      NULL, LastWinError,
      //LANG_SYSTEM_DEFAULT,
      MAKELANGID(LANG_NEUTRAL,SUBLANG_DEFAULT),
      (char *)BufferPtr, BufferSize, NULL);
 // change error message for serial ports
 if(lstrcmp(BufferPtr,(char *)"The system cannot find the file specified.\r\n")==0)
   lstrcpy(BufferPtr,(char *)"The system cannot find the port specified.\r\n");

 // pop up message box if passed size is 0
 //if(Size==0) MessageBox(NULL,BufferPtr,(char *)"SioWinError",MB_TASKMODAL|MB_ICONEXCLAMATION);
 return (int) LastWinError;
}

//---------------------------------------------------------
//
// get text associated with error code

NoMangle int DLL_IMPORT_EXPORT SioErrorText(int ErrCode, char *Buffer, int BufLen)
{int i;
 int ErrorCode;
 char *ErrorPtr;
 int Copied = 0;
 if(Buffer==NULL) return WSC_BUFFER_RANGE;
 if(ErrCode==WSC_WIN32ERR) return SioWinError(Buffer, BufLen);
 // search table
 for(i=0;;i++)
   {ErrorCode = ErrorList[i].Code;
    if(ErrorCode==999) break;
    if(ErrorCode==ErrCode)
      {// found error code
       ErrorPtr = ErrorList[i].Text;
       // copy text to caller's buffer
       while(1)
          {if(Copied>=BufLen-1) break;
           if(*ErrorPtr=='\0') break;
           *Buffer++ = *ErrorPtr++;
           Copied++;
          }
       *Buffer = '\0';
       return Copied;
      }
   }
 // error code not found
 *Buffer = '\0';
 return 0;
}


//---------------------------------------------------------
// no longer supported

NoMangle int DLL_IMPORT_EXPORT SioRead(int Port, int Reg)
{
 return -1;
}

//---------------------------------------------------------
// get stsyem ticks

NoMangle DWORD DLL_IMPORT_EXPORT SioTimer(void)
{
 return GetTickCount();
}

//---------------------------------------------------------
// returns the Windows Tick Count (# milliseconds since bootup)

NoMangle DWORD DLL_IMPORT_EXPORT SioTimeMark(DWORD Mask)
{if(Mask==0)return GetTickCount();
 else return (Mask & GetTickCount());
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioEventWait(int Port, DWORD MaskArg, DWORD Timeout)
{HANDLE hComm;
 int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 // save event mask
 PortData[Port].Mask = MaskArg;
 // block on event mask
 Code = oioEventStart(Port, (LPDWORD)&MaskArg);
 if(Code!=WSC_IO_ERROR) Code = oioEventWait(Port, Timeout);
 return Code;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioEventChar(int Port, char EvtChar, DWORD Timeout)
{DCB  *DCBptr;
 HANDLE hComm;
 int Code;
 if((Code=VerifyPort(Port))<0) return Code;
 DCBptr = PortData[Port].DCBptr;
 hComm = PortData[Port].ComHandle;
 // get comm state
 DCBptr->DCBlength = sizeof(struct _DCB);
 if(!GetCommState(hComm,DCBptr))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 // set event character
 DCBptr->EvtChar = EvtChar;
 // now set the DCB
 if(!SetCommState(hComm,DCBptr))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 return SioEventWait(Port, EV_RXFLAG, Timeout);
}

//---------------------------------------------------------
//

NoMangle int DLL_IMPORT_EXPORT SioEvent(int Port, DWORD MaskArg)
{
 return SioEventWait(Port, MaskArg, INFINITE);
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioMessage(int Port, HWND hMsgWnd, WORD MsgCode, DWORD Mask)
{HANDLE hComm;
 int Code;
 DWORD ThreadID;
 HANDLE ThreadHandle;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 // acquire event mutex
    ///WaitForSingleObject(PortData[Port].EventMutex, INFINITE);
 Sleep(0);
 // save parameters
 PortData[Port].hMsgWnd = hMsgWnd;
 PortData[Port].Mask = Mask;
 ///PortData[Port].Port = Port;
 PortData[Port].MsgCode = MsgCode;
 if((HWND)hMsgWnd==0)
    {// unblock SioEvent
     oioEventSignal(Port);
     return 1;
    }
 // MsgThread already running ?
 Sleep(0); // this sleep is necessary!
 if(PortData[Port].MsgHandle)
   {   ///ReleaseMutex(PortData[Port].EventMutex);
    return 1;
   }
 // start message thread [the thread's stack is freed when it exits but not if the thread is terminated by another thread.]
 ThreadHandle = CreateThread(0,1024,(LPTHREAD_START_ROUTINE)MsgThread,(void *)&PortData[Port].Port,0,&ThreadID);
 if(ThreadHandle==0)
   {if(EnableMsgBox)
      {MsgBox((char *)"Cannot create message thread");
         ///ReleaseMutex(PortData[Port].EventMutex);
       return WSC_THREAD;
      }
   }
 PortData[Port].MsgHandle = ThreadHandle;
 // set thread priority
 SetThreadPriority(ThreadHandle,THREAD_PRIORITY_TIME_CRITICAL-1);
 // release event mutex
    ///ReleaseMutex(PortData[Port].EventMutex);
 return 0;
}

//---------------------------------------------------------

// convert (null terminated) 16-bit (ascii) char buffer to 8-bit

NoMangle int DLL_IMPORT_EXPORT SioShortToByte(char *Ptr)
{int i;
 char c;
 for(i=0;;i++)
   {c = Ptr[i+i];
    Ptr[i] = c;
    if(c=='\0') break;
   }
 return i;
}

//---------------------------------------------------------

// convert (null terminated) 8-bit char buffer to 16-bit (ascii) chars

NoMangle int DLL_IMPORT_EXPORT SioByteToShort(char *Ptr)
{int i;
 int Length = lstrlen(Ptr);
 for(i=Length-1;i>=0;i--)
   {Ptr[i+i] = Ptr[i];
    Ptr[i+i+1] = '\0';
   }
 return Length;
}

//---------------------------------------------------------
//
// WSC READ INTERVAL TIMEOUT (DWORD ReadInter)
//
//   Sets the maximum period of time (in milliseconds) allowed between
//   two sequential bytes being read from the serial port before the
//   receive operation terminates.
//
//   If set to MAXDWORD and the other two above READ timeouts are set to
//   zero, then serial receive calls return immediately without waiting.
//
// WSC READ TIMEOUT MULTIPLIER (DWORD ReadMult)
//
//   Sets the multiplier (in milliseconds) used to calculate the overall
//   timeout of serial receive operations. This timeout is given by:
//
//   NbrBytes * ReadTotalTimeoutMultiplier + ReadTotalTimeoutConstant
//
//   where NbrBytes = number of bytes requested.
//
// WSC READ TIMEOUT CONSTANT (DWORD ReadCons)
//
//   Sets the constant (in milliseconds) used to calculate the overall
//   timeout of serial receive operations. This timeout is given by:
//
//   NbrBytes * ReadTotalTimeoutMultiplier + ReadTotalTimeoutConstant
//
//   where NbrBytes = number of bytes requested.
//
// WSC WRITE TIMEOUT MULTIPLIER (DWORD WriteMult)
//
//   Sets the multiplier (in milliseconds) used to calculate the overall
//   timeout of serial transmit operations. This timeout is given by:
//
//   NbrBytes * WriteTotalTimeoutMultiplier + WriteTotalTimeoutConstant
//
//   where NbrBytes =  number of bytes requested.
//
// WSC WRITE TIMEOUT CONSTANT (DWORD WriteCons)
//
//   Sets the constant (in milliseconds) used to calculate the overall
//   timeout of serial transmit operations. This timeout is given by:
//
//   NbrBytes * WriteTotalTimeoutMultiplier + WriteTotalTimeoutConstant
//
//   where NbrBytes =  number of bytes requested.
//
//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioSetTimeouts(int Port,
                            DWORD ReadInter,   // read interval t/o
                            DWORD ReadMult,    // read t/o multiplier
                            DWORD ReadCons,    // read t/o constant
                            DWORD WriteMult,   // write t/o multiplier
                            DWORD WriteCons)   // write t/o constant
{int Code;
 HANDLE hComm;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 if(ReadInter==(DWORD)-1) ReadInter = MAXDWORD;
 PortData[Port].Timeouts.ReadIntervalTimeout = ReadInter;
 PortData[Port].Timeouts.ReadTotalTimeoutMultiplier = ReadMult;
 PortData[Port].Timeouts.ReadTotalTimeoutConstant = ReadCons;
 PortData[Port].Timeouts.WriteTotalTimeoutMultiplier = WriteMult;
 PortData[Port].Timeouts.WriteTotalTimeoutConstant = WriteCons;
 // set new timeouts
 if(!SetCommTimeouts(hComm, &PortData[Port].Timeouts))
   {LastWinError = GetLastError();
    return WSC_WIN32ERR;
   }
 else return 1;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioSleep(int MilliSecs)
{
 Sleep(MilliSecs);
 return 1;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioHexView(char *Binary, int ByteCount, char *Buffer, int BufLen)
{int i;
 int Last;
 char *Ptr;
 if((ByteCount<=0)|| (BufLen<=0)) return 0;
 if((3*ByteCount)>BufLen) return WSC_BUFLEN_RANGE;
 Ptr = (char *)&Buffer[0];
 for(i=0;i<ByteCount;i++)
   {wsprintf(Ptr, "%02x ", (BYTE)Binary[i]);
    Ptr += 3;
   }
 Last = -1 + 3*ByteCount;
 Buffer[Last] = '\0';
 return Last;
}

//---------------------------------------------------------

NoMangle int DLL_IMPORT_EXPORT SioRxWait(int Port, unsigned int BytesWanted, unsigned int Timeout)
{int Code;
 unsigned int SleepTime;
 unsigned int cps;  // chars per second
 HANDLE hComm;
 DWORD Errors;
 unsigned int TimeMark = 0;
 unsigned int BytesInQueue;
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 // assume 10 bits per byte (with start, stop, and parity bits)
 cps = PortDCB[Port].BaudRate / 10;
 while(1)
    {// enough incoming bytes ?
     if(ClearCommError(hComm,&Errors,&ComStat)) BytesInQueue = ComStat.cbInQue;
     else BytesInQueue = 0;
     if(BytesInQueue >= BytesWanted) return BytesInQueue;
     // check time
     if(TimeMark==0) TimeMark = GetCurrentTime() + Timeout;
     if(GetCurrentTime()>TimeMark) return WSC_TIMEOUT;
     // compute how long to sleep
     SleepTime = (1000 * (BytesWanted - BytesInQueue)) / cps;
     // adjust sleep to [1,250] mSec
     if(SleepTime<1) SleepTime = 1;
     else if(SleepTime>250) SleepTime = 250;
     Sleep(SleepTime);
    } // end-while
}

//---------------------------------------------------------
//
// Will return 0 if there is no incoming data within 'QuietTime' milliseconds
// Will return WSC_TIMEOUT if timeout period expires before quiet period is seen.
// All incoming data is cleared

NoMangle int DLL_IMPORT_EXPORT SioQuiet(int Port, unsigned int QuietTime, unsigned int TimeOut)
{int Code;
 unsigned int TimeOutMark;
 unsigned int QuietTimeMark;
 HANDLE hComm;
 // verify the port
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 QuietTimeMark = GetCurrentTime() + QuietTime;
 TimeOutMark = GetCurrentTime() + TimeOut;
 while(1)
   {if(GetCurrentTime()>=TimeOutMark) return WSC_TIMEOUT;
    // anything incoming ?
    Code = SioRxQue(Port);
    if(Code<0) return Code;
    if(Code>0)
      {// clear all incoming
       SioRxClear(Port);
       // update quiet mark
       QuietTimeMark = GetCurrentTime() + QuietTime;
       Sleep(1);
      }
    // code == 0
    if(GetCurrentTime()>=QuietTimeMark) return 0;
   }
}

//---------------------------------------------------------
//
// will wait 'Timeout' milliseconds for an incoming byte

NoMangle int DLL_IMPORT_EXPORT SioWaitFor(int Port, unsigned int TimeOut)
{int Code;
 unsigned int TimeMark;
 unsigned int SleepTime;
 HANDLE hComm;
 unsigned int BaudRate;
 // verify the port
 if((Code=VerifyPort(Port))<0) return Code;
 hComm = PortData[Port].ComHandle;
 if(hComm==IHV) return WSC_IE_NOPEN;
 TimeMark = GetCurrentTime() + TimeOut;
 BaudRate = PortDCB[Port].BaudRate;
 if(BaudRate==0) return WSC_IE_BAUDRATE;
 // compute sleep time
 SleepTime = 10000 / BaudRate;
 if(SleepTime<1) SleepTime = 1;
 while(1)
   {Code = SioGetc(Port);
    if(Code==WSC_NO_DATA)
      {if(GetCurrentTime()>=TimeMark) return WSC_TIMEOUT;
       SioSleep(SleepTime);
      }
    else return Code;
   }
}

//---------------------------------------------------------
//
// Computes the longitudinal parity check byte
//
//     // ISO 1155
//     Set LRC = 0
//     For each byte b in the buffer
//     do
//       Set LRC = (LRC + b) AND 0xFF
//     end do
//     Set LRC = (((LRC XOR 0xFF) + 1) AND 0xFF)

NoMangle int DLL_IMPORT_EXPORT SioLRC(char *Buffer, int BufLen)
{int i;
 BYTE CheckSum = 0;
 for(i=0;i<BufLen;i++) CheckSum += *Buffer++;
 CheckSum = 0x0ff & ((CheckSum ^ 0x0ff) + 1);
 return CheckSum;
}

//---------------------------------------------------------
//
// Computes 16-bit CCITT CRC. Uses polynomial 0x1021 (reversed 0x8408)
// SioCRC16("ABC") = 0x3994


NoMangle USHORT DLL_IMPORT_EXPORT SioCRC16(BYTE *BufPtr, int BufLen)
{int i;
 USHORT CRC = 0;
 // compute 16-bit CRC over passed buffer
 for(i=0;i<BufLen;i++) CRC = (CRC << 8) ^ CRC16Table[ (CRC >> 8) ^ (*BufPtr++)];
 return CRC;
}

//---------------------------------------------------------
//
// Computes 32-bit CCITT CRC. Uses polynomial 0x04C11DB7 (reversed 0xEDB88320)
// SioCRC32("ABC") = 0xA3830348
//

NoMangle UINT DLL_IMPORT_EXPORT SioCRC32(BYTE *BufPtr, int BufLen)
{int i;
 UINT c = 0 ^ 0xffffffff;
 // compute 32-bit CRC over passed buffer
 for(i=0;i<BufLen;i++) c = CRC32Table[(c ^ BufPtr[i]) & 0xff] ^ (c >> 8);
 return c ^ 0xffffffff;
}


//---------------------------------------------------------
//#define WSC_COMPUTE_CPS 0x01
//#define WSC_TEST_BUFFER 0x10
//---------------------------------------------------------

//static int CompareBuffers(char *Ptr1, char *Ptr2, int Size)
//{int i;
// int Count = 0;
// char c1, c2;
// for(i=0;i<Size;i++)
//   {c1 = *Ptr1++;
//    c2 = *Ptr2++;
//    if(c1==c2) Count++;
//   }
// return Count;
//}


//---------------------------------------------------------
