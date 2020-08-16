//
//	Audio interface Routine

//	Passes audio samples to/from the sound interface

//	As this is platform specific it also has the main() routine, which does
//	platform specific initialisation before calling ardopmain()

//	This is ALSASound.c for Linux
//	Windows Version is Waveout.c
//	Nucleo Version is NucleoSound.c


#include <signal.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <CoreAudio/CoreAudio.h>

#include "ARDOPC.h"

#define SHARECAPTURE		// if defined capture device is opened and closed for each transission

#define HANDLE int

void gpioSetMode(unsigned gpio, unsigned mode);
void gpioWrite(unsigned gpio, unsigned level);
int WriteLog(char * msg, int Log);
int _memicmp(unsigned char *a, unsigned char *b, int n);
int stricmp(const unsigned char * pStr1, const unsigned char *pStr2);
int gpioInitialise(void);
HANDLE OpenCOMPort(VOID * pPort, int speed, BOOL SetDTR, BOOL SetRTS, BOOL Quiet, int Stopbits);

VOID COMSetDTR(HANDLE fd);
VOID COMClearDTR(HANDLE fd);
VOID COMSetRTS(HANDLE fd);
VOID COMClearRTS(HANDLE fd);
VOID RadioPTT(int PTTState);
VOID SerialHostPoll();
VOID TCPHostPoll();
int CloseSoundCard();
int PackSamplesAndSend(short * input, int nSamples);
void displayLevel(int max);
BOOL WriteCOMBlock(HANDLE fd, char * Block, int BytesToWrite);
VOID processargs(int argc, char * argv[]);

int initdisplay();

extern BOOL blnDISCRepeating;
extern BOOL UseKISS;			// Enable Packet (KISS) interface


void Sleep(int mS)
{
	usleep(mS * 1000);
	return;
}


// Windows and ALSA work with signed samples +- 32767
// STM32 and Teensy DAC uses unsigned 0 - 4095

short buffer[2][1200];			// Two Transfer/DMA buffers of 0.1 Sec
short inbuffer[2][1200];		// Two Transfer/DMA buffers of 0.1 Sec

BOOL Loopback = FALSE;
//BOOL Loopback = TRUE;

char CaptureDevice[80] = "ARDOP";
char PlaybackDevice[80] = "ARDOP";

char * CaptureDevices = CaptureDevice;
char * PlaybackDevices = CaptureDevice;

void InitSound();

int Ticks;

int LastNow;

extern int Number;				// Number waiting to be sent

#include <stdarg.h>

FILE *logfile[3] = {NULL, NULL, NULL};
char LogName[3][20] = {"ARDOPDebug", "ARDOPException", "ARDOPSession"};

#define DEBUGLOG 0
#define EXCEPTLOG 1
#define SESSIONLOG 2

FILE *statslogfile = NULL;

VOID CloseDebugLog()
{	
	if (logfile[DEBUGLOG])
		fclose(logfile[DEBUGLOG]);
	logfile[DEBUGLOG] = NULL;
}

VOID CloseStatsLog()
{
	if (statslogfile)
		fclose(statslogfile);
	statslogfile = NULL;
}


VOID Debugprintf(const char * format, ...)
{
	char Mess[10000];
	va_list(arglist);

	va_start(arglist, format);
	vsnprintf(Mess, sizeof(Mess), format, arglist);
	strcat(Mess, "\r\n");

	printf("%s", Mess);
	WriteLog(Mess, DEBUGLOG);
	return;
}

VOID WriteDebugLog(int Level, const char * format, ...)
{
	char Mess[10000];
	va_list(arglist);

	va_start(arglist, format);
	vsnprintf(Mess, sizeof(Mess), format, arglist);
	strcat(Mess, "\n");
	
	if (Level <= ConsoleLogLevel)
		printf("%s", Mess);

	if (!DebugLog)
		return;

	WriteLog(Mess, DEBUGLOG);
	return;
}

VOID WriteExceptionLog(const char * format, ...)
{
	char Mess[10000];
	va_list(arglist);

	va_start(arglist, format);
	vsnprintf(Mess, sizeof(Mess), format, arglist);
	strcat(Mess, "\n");

	printf("%s", Mess);
	WriteLog(Mess, EXCEPTLOG);

	fclose(logfile[EXCEPTLOG]);
	logfile[EXCEPTLOG] = NULL;
	return;
}

VOID Statsprintf(const char * format, ...)
{
	char Mess[10000];
	va_list(arglist);
	UCHAR Value[100];
	char timebuf[32];
	struct timespec tp;

	int hh;
	int mm;
	int ss;

	clock_gettime(CLOCK_REALTIME, &tp);

	va_start(arglist, format);
	vsnprintf(Mess, sizeof(Mess), format, arglist);
	strcat(Mess, "\n");

	ss = tp.tv_sec % 86400;		// Secs int day
	hh = ss / 3600;
	mm = (ss - (hh * 3600)) / 60;
	ss = ss % 60;

	sprintf(timebuf, "%02d:%02d:%02d.%03d ",
		hh, mm, ss, (int)tp.tv_nsec/1000000);

	if (statslogfile == NULL)
	{
		struct tm * tm;
		time_t T;

		T = time(NULL);
		tm = gmtime(&T);

		sprintf(Value, "%s%d_%04d%02d%02d.log",
		LogName[2], port, tm->tm_year +1900, tm->tm_mon+1, tm->tm_mday);

		if ((statslogfile = fopen(Value, "ab")) == NULL)
		{
			perror(Value);
			return;
		}
		else
		{
			fputs(timebuf, statslogfile);
			fputs("\n", statslogfile);
		}

	}

	fputs(Mess, statslogfile);
	printf("%s", Mess);

	return;
}

void printtick(char * msg)
{
	Debugprintf("%s %i", msg, Now - LastNow);
	LastNow = Now;
}

struct timespec time_start;

unsigned int getTicks()
{	
	struct timespec tp;
	
	clock_gettime(CLOCK_MONOTONIC, &tp);
	return (tp.tv_sec - time_start.tv_sec) * 1000 + (tp.tv_nsec - time_start.tv_nsec) / 1000000;
}

void PlatformSleep()
{
	Sleep(1);
		
	if (PKTLEDTimer && Now > PKTLEDTimer)
    {
      PKTLEDTimer = 0;
      SetLED(PKTLED, 0);				// turn off packet rxed led
    }
}

// PTT via GPIO code

#ifdef __ARM_ARCH

#define PI_INPUT  0
#define PI_OUTPUT 1
#define PI_ALT0   4
#define PI_ALT1   5
#define PI_ALT2   6
#define PI_ALT3   7
#define PI_ALT4   3
#define PI_ALT5   2

// Set GPIO pin as output and set low

extern int pttGPIOPin;
extern BOOL pttGPIOInvert;



void SetupGPIOPTT()
{
	if (pttGPIOPin == -1)
	{
		WriteDebugLog(LOGALERT, "GPIO PTT disabled"); 
		RadioControl = FALSE;
		useGPIO = FALSE;
	}
	else
	{
		if (pttGPIOPin < 0) {
			pttGPIOInvert = TRUE;
			pttGPIOPin = -pttGPIOPin;
		}

		gpioSetMode(pttGPIOPin, PI_OUTPUT);
		gpioWrite(pttGPIOPin, pttGPIOInvert ? 1 : 0);
		WriteDebugLog(LOGALERT, "Using GPIO pin %d for PTT", pttGPIOPin); 
		RadioControl = TRUE;
		useGPIO = TRUE;
	}
}
#endif


static void sigterm_handler(int sig)
{
	printf("terminating on SIGTERM\n");
	blnClosing = TRUE;
}

static void sigint_handler(int sig)
{
	printf("terminating on SIGINT\n");
	blnClosing = TRUE;
}

char * PortString = NULL;


void main(int argc, char * argv[])
{
	struct timespec tp;
	struct sigaction act;

//	Sleep(1000);	// Give LinBPQ time to complete init if exec'ed by linbpq

	processargs(argc, argv);

	Debugprintf("ARDOPC Version %s", ProductVersion);

	if (HostPort[0])
	{		
		char *pkt = strlop(HostPort, '/');

		if (_memicmp(HostPort, "COM", 3) == 0)
		{
			SerialMode = 1;
		}
		else
			port = atoi(HostPort);

		if (pkt)
			pktport = atoi(pkt);
	}

	
	if (CATPort[0])
	{
		char * Baud = strlop(CATPort, ':');
		if (Baud)
			CATBAUD = atoi(Baud);
		hCATDevice = OpenCOMPort(CATPort, CATBAUD, FALSE, FALSE, FALSE, 0);
	}

	if (PTTPort[0])
	{
		char * Baud = strlop(PTTPort, ':');
		char * pin = strlop(PTTPort, '=');
	
		if (Baud)
			PTTBAUD = atoi(Baud);

		if (strcmp(CATPort, PTTPort) == 0)
		{
			hPTTDevice = hCATDevice;
		}
		else
		{
			if (stricmp(PTTPort, "GPIO") == 0)
			{
				// Initialise GPIO for PTT if available

#ifdef __ARM_ARCH

				if (gpioInitialise() == 0)
				{	
					printf("GPIO interface for PTT available\n");
					gotGPIO = TRUE;

					if (pin)
						pttGPIOPin = atoi(pin);
					else
						pttGPIOPin = 17;
				
					SetupGPIOPTT();
			   }
				else
					printf("Couldn't initialise GPIO interface for PTT\n");

#else
			printf("GPIO interface for PTT not available on this platform\n");
#endif

			}
			else		//  Not GPIO
			{
				hPTTDevice = OpenCOMPort(PTTPort, PTTBAUD, FALSE, FALSE, FALSE, 0);
	
			}
		}
	}

	if (hCATDevice)
	{
		WriteDebugLog(LOGALERT, "CAT Control on port %s", CATPort); 
		COMSetRTS(hPTTDevice);
		COMSetDTR(hPTTDevice);
		if (PTTOffCmdLen)
		{
			WriteDebugLog(LOGALERT, "PTT using CAT Port", CATPort); 
			RadioControl = TRUE;
		}
	}
	else
	{
		// Warn of -u and -k defined but no CAT Port

		if (PTTOffCmdLen)
		{
			WriteDebugLog(LOGALERT, "Warning PTT Off string defined but no CAT port", CATPort); 
		}
	}

	if (hPTTDevice)
	{
		WriteDebugLog(LOGALERT, "Using RTS on port %s for PTT", PTTPort); 
		COMClearRTS(hPTTDevice);
		COMClearDTR(hPTTDevice);
		RadioControl = TRUE;
	}	


	initdisplay();

	if (SerialMode)
		Debugprintf("ARDOPC Using a pseudotty symlinked to %s", HostPort);
	else
		Debugprintf("ARDOPC listening on port %d", port);

	if (UseKISS && pktport)
		Debugprintf("ARDOPC listening for KISS frames on port %d", pktport);

	// Get Time Reference
		
	clock_gettime(CLOCK_MONOTONIC, &time_start);
	LastNow = getTicks();

	// Trap signals

	memset (&act, '\0', sizeof(act));
 
	act.sa_handler = &sigint_handler; 
	if (sigaction(SIGINT, &act, NULL) < 0) 
		perror ("SIGINT");

	act.sa_handler = &sigterm_handler; 
	if (sigaction(SIGTERM, &act, NULL) < 0) 
		perror ("SIGTERM");

	act.sa_handler = SIG_IGN; 

	if (sigaction(SIGHUP, &act, NULL) < 0) 
		perror ("SIGHUP");

	if (sigaction(SIGPIPE, &act, NULL) < 0) 
		perror ("SIGPIPE");

	ardopmain();

	// if PTY used, remove it

	if (SerialMode)
		unlink (HostPort);

}

void txSleep(int mS)
{
	// called while waiting for next TX buffer or to delay response.
	// Run background processes

	if (SerialMode)
		SerialHostPoll();
	else
		TCPHostPoll();

	Sleep(mS);

	if (PKTLEDTimer && Now > PKTLEDTimer)
    {
      PKTLEDTimer = 0;
      SetLED(PKTLED, 0);				// turn off packet rxed led
    }
}

// ALSA Code 

#define true 1
#define false 0

int m_playchannels = 1;
int m_recchannels = 1;

BOOL UseLeft = TRUE;
BOOL UseRight = TRUE;

char SavedCaptureDevice[256];	// Saved so we can reopen
char SavedPlaybackDevice[256];

int Savedplaychannels = 1;

int SavedCaptureRate;
int SavedPlaybackRate;

// This rather convoluted process simplifies marshalling from Managed Code

char ** WriteDevices = NULL;
int WriteDeviceCount = 0;

char ** ReadDevices = NULL;
int ReadDeviceCount = 0;

// Routine to check that library is available

int CheckifLoaded()
{
	// Prevent CTRL/C from closing the TNC
	// (This causes problems if the TNC is started by LinBPQ)

	signal(SIGHUP, SIG_IGN);
	signal(SIGINT, SIG_IGN);
	signal(SIGPIPE, SIG_IGN);

	return TRUE;
}

int GetOutputDeviceCollection()
{
	return 0;
}


int GetInputDeviceCollection()
{
	return 0;
}
int OpenSoundPlayback(char * PlaybackDevice, int m_sampleRate, int channels, char * ErrorMsg)
{
	return true;
}

int OpenSoundCapture(char * CaptureDevice, int m_sampleRate, char * ErrorMsg)
{
 	return TRUE;
}

int OpenSoundCard(char * CaptureDevice, char * PlaybackDevice, int c_sampleRate, int p_sampleRate, char * ErrorMsg)
{
	return false;
}



int CloseSoundCard()
{
	return 0;
}


int SoundCardWrite(short * input, unsigned int nSamples)
{
	return 0;
}

int PackSamplesAndSend(short * input, int nSamples)
{
	return 0;
}

int SoundCardRead(short * input, unsigned int nSamples)
{
	return 0;
}




int PriorSize = 0;

int Index = 0;				// DMA Buffer being used 0 or 1
int inIndex = 0;				// DMA Buffer being used 0 or 1

BOOL DMARunning = FALSE;		// Used to start DMA on first write

short * SendtoCard(short * buf, int n)
{
	return 0;
}

short loopbuff[1200];		// Temp for testing - loop sent samples to decoder


//		// This generates a nice musical pattern for sound interface testing
//    for (t = 0; t < sizeof(buffer); ++t)
//        buffer[t] =((((t * (t >> 8 | t >> 9) & 46 & t >> 8)) ^ (t & t >> 13 | t >> 6)) & 0xFF);



void InitSound(BOOL Quiet)
{
	GetInputDeviceCollection();
	GetOutputDeviceCollection();
	
	OpenSoundCard(CaptureDevice, PlaybackDevice, 12000, 12000, NULL);
}

int min = 0, max = 0, lastlevelreport = 0, lastlevelGUI = 0;
UCHAR CurrentLevel = 0;		// Peak from current samples


void PollReceivedSamples()
{
	// Process any captured samples
	// Ideally call at least every 100 mS, more than 200 will loose data

	if (SoundCardRead(&inbuffer[0][0], ReceiveSize))
	{
		// returns ReceiveSize or none

		short * ptr = &inbuffer[0][0];
		int i;

		for (i = 0; i < ReceiveSize; i++)
		{
			if (*(ptr) < min)
				min = *ptr;
			else if (*(ptr) > max)
				max = *ptr;
			ptr++;
		}

		displayLevel(max);
		CurrentLevel = ((max - min) * 75) /32768;	// Scale to 150 max

		if ((Now - lastlevelGUI) > 2000)	// 2 Secs
		{
			if (WaterfallActive == 0 && SpectrumActive == 0)				// Don't need to send as included in Waterfall Line
				SendtoGUI('L', &CurrentLevel, 1);	// Signal Level
			
			lastlevelGUI = Now;

			if ((Now - lastlevelreport) > 10000)	// 10 Secs
			{
				char HostCmd[64];
				lastlevelreport = Now;

				sprintf(HostCmd, "INPUTPEAKS %d %d", min, max);
				SendCommandToHostQuiet(HostCmd);

				WriteDebugLog(LOGDEBUG, "Input peaks = %d, %d", min, max);
			}
			min = max = 0;							// Every 2 secs
		}

		if (Capturing && Loopback == FALSE)
			ProcessNewSamples(&inbuffer[0][0], ReceiveSize);
	}
} 

void StopCapture()
{
	Capturing = FALSE;

#ifdef SHARECAPTURE

	// Stopcapture is only called when we are about to transmit, so use it to open plaback device. We don't keep
	// it open all the time to facilitate sharing.

	OpenSoundPlayback(SavedPlaybackDevice, SavedPlaybackRate, Savedplaychannels, NULL);
#endif
}

void StartCodec(char * strFault)
{
	strFault[0] = 0;
	OpenSoundCard(CaptureDevice, PlaybackDevice, 12000, 12000, strFault);
}

void StopCodec(char * strFault)
{
	strFault[0] = 0;
	CloseSoundCard();
}

void StartCapture()
{
	Capturing = TRUE;
	DiscardOldSamples();
	ClearAllMixedSamples();
	State = SearchingForLeader;

//	Debugprintf("Start Capture");
}
void CloseSound()
{ 
	CloseSoundCard();
}

int WriteLog(char * msg, int Log)
{
	FILE *file;
	char timebuf[128];
	struct timespec tp;

	UCHAR Value[100];
	
	int hh;
	int mm;
	int ss;

	clock_gettime(CLOCK_REALTIME, &tp);
	
	if (logfile[Log] == NULL)
	{
		struct tm * tm;
		time_t T;

		T = time(NULL);
		tm = gmtime(&T);

		if (HostPort[0])
			sprintf(Value, "%s%s_%04d%02d%02d.log",
				LogName[Log], HostPort, tm->tm_year +1900, tm->tm_mon+1, tm->tm_mday);
		else
			sprintf(Value, "%s%d_%04d%02d%02d.log",
				LogName[Log], port, tm->tm_year +1900, tm->tm_mon+1, tm->tm_mday);
	
		if ((logfile[Log] = fopen(Value, "a")) == NULL)
			return FALSE;
	}
	ss = tp.tv_sec % 86400;		// Secs int day
	hh = ss / 3600;
	mm = (ss - (hh * 3600)) / 60;
	ss = ss % 60;

	sprintf(timebuf, "%02d:%02d:%02d.%03d ",
		hh, mm, ss, (int)tp.tv_nsec/1000000);

	fputs(timebuf, logfile[Log]);
	fputs(msg, logfile[Log]);
	fflush(logfile[Log]);
	return 0;
}






VOID WriteSamples(short * buffer, int len)
{

#ifdef WIN32
	fwrite(buffer, 1, len * 2, wavfp1);
#endif
}

unsigned short * SoundInit()
{
	Index = 0;
	return &buffer[0][0];
}
	
//	Called at end of transmission

void SoundFlush()
{
	return;
}

VOID RadioPTT(int PTTState)	
{
#ifdef __ARM_ARCH
	if (useGPIO)
		gpioWrite(pttGPIOPin, (pttGPIOInvert ? (1-PTTState) : (PTTState)));
	else
#endif
	{
		if (PTTMode & PTTRTS)
			if (PTTState)
				COMSetRTS(hPTTDevice);
			else
				COMClearRTS(hPTTDevice);

		if (PTTMode & PTTDTR)
			if (PTTState)
				COMSetDTR(hPTTDevice);
			else
				COMClearDTR(hPTTDevice);

		if (PTTMode & PTTCI_V)
			if (PTTState)
				WriteCOMBlock(hCATDevice, PTTOnCmd, PTTOnCmdLen);
			else
				WriteCOMBlock(hCATDevice, PTTOffCmd, PTTOffCmdLen);
	}
}

//  Function to send PTT TRUE or PTT FALSE commanad to Host or if local Radio control Keys radio PTT 

const char BoolString[2][6] = {"FALSE", "TRUE"};

BOOL KeyPTT(BOOL blnPTT)
{
	// Returns TRUE if successful False otherwise

	if (blnLastPTT &&  !blnPTT)
		dttStartRTMeasure = Now;	 // start a measurement on release of PTT.

	if (!RadioControl)
		if (blnPTT)
			SendCommandToHostQuiet("PTT TRUE");
		else
			SendCommandToHostQuiet("PTT FALSE");

	else
		RadioPTT(blnPTT);

	WriteDebugLog(LOGDEBUG, "[Main.KeyPTT]  PTT-%s", BoolString[blnPTT]);

	blnLastPTT = blnPTT;
	SetLED(0, blnPTT);
	return TRUE;
}


static struct speed_struct
{
	int	user_speed;
	speed_t termios_speed;
} speed_table[] = {
	{300,         B300},
	{600,         B600},
	{1200,        B1200},
	{2400,        B2400},
	{4800,        B4800},
	{9600,        B9600},
	{19200,       B19200},
	{38400,       B38400},
	{57600,       B57600},
	{115200,      B115200},
	{-1,          B0}
};


int stricmp(const unsigned char * pStr1, const unsigned char *pStr2)
{
    unsigned char c1, c2;
    int  v;

	if (pStr1 == NULL)
	{
		if (pStr2)
			Debugprintf("stricmp called with NULL 1st param - 2nd %s ", pStr2);
		else
			Debugprintf("stricmp called with two NULL params");

		return 1;
	}


    do {
        c1 = *pStr1++;
        c2 = *pStr2++;
        /* The casts are necessary when pStr1 is shorter & char is signed */
        v = tolower(c1) - tolower(c2);
    } while ((v == 0) && (c1 != '\0') && (c2 != '\0') );

    return v;
}

char Leds[8]= {0};
unsigned int PKTLEDTimer = 0;

void SetLED(int LED, int State)
{
	// If GUI active send state

	Leds[LED] = State;
	SendtoGUI('D', Leds, 8);
}

void DrawTXMode(const char * Mode)
{
	unsigned char Msg[64];
	strcpy(Msg, Mode);
	SendtoGUI('T', Msg, strlen(Msg) + 1);		// TX Frame

}

void DrawTXFrame(const char * Frame)
{
	unsigned char Msg[64];
	strcpy(Msg, Frame);
	SendtoGUI('T', Msg, strlen(Msg) + 1);		// TX Frame
}

void DrawRXFrame(int State, const char * Frame)
{
	unsigned char Msg[64];

	Msg[0] = State;				// Pending/Good/Bad
	strcpy(&Msg[1], Frame);
	SendtoGUI('R', Msg, strlen(Frame) + 1);	// RX Frame
}
UCHAR Pixels[4096];
UCHAR * pixelPointer = Pixels;


void mySetPixel(unsigned char x, unsigned char y, unsigned int Colour)
{
	// Used on Windows for constellation. Save points and send to GUI at end
	
	*(pixelPointer++) = x;
	*(pixelPointer++) = y;
	*(pixelPointer++) = Colour;
}
void clearDisplay()
{
	// Reset pixel pointer

	pixelPointer = Pixels;

}
void updateDisplay()
{
//	 SendtoGUI('C', Pixels, pixelPointer - Pixels);	
}
void DrawAxes(int Qual, const char * Frametype, char * Mode)
{
	UCHAR Msg[80];

	// Teensy used Frame Type, GUI Mode
	
	SendtoGUI('C', Pixels, pixelPointer - Pixels);	
	pixelPointer = Pixels;

	sprintf(Msg, "%s Quality: %d", Mode, Qual);
	SendtoGUI('Q', Msg, strlen(Msg) + 1);	
}
void DrawDecode(char * Decode)
{}




