//
// LS1 and LS2 acoustic recorders
// THIS VERSION ONLY WORKS WITH 1 CARD with Version 4 LS1/L2 Boards
//
// Loggerhead Instruments
// 2026 For use with LS1-LS2 Version 4 ONLY
// David Mann 

// 
// Modified from PJRC audio code
// http://www.pjrc.com/store/teensy3_audio.html
//
// Compile with 96 MHz Fastest

// Modified by WMXZ 15-05-2018 for SdFS and multiple sampling frequencies
// Optionally uses SdFS from Bill Greiman https://github.com/greiman/SdFs; but has higher current draw in sleep

//*****************************************************************************************

char codeVersion[5] = "4.42";
static boolean printDiags = 0;  // 1: serial print diagnostics; 0: no diagnostics
static boolean sleepDiags = 1;  // 1: bench diagnostics (diag.ino): wake log and no-audio report on USB serial; 0 for deployment
#define MQ 100 // to be used with LHI record queue (modified local version)
int roundSeconds = 60;//start time modulo to nearest roundSeconds
int wakeahead = 4;  //wake from snooze to give hydrophone to power up
int noDC = 0; // 0 = freezeDC offset; 1 = remove DC offset; 2 = bypass
int NCHAN = 2;
//*****************************************************************************************


#include "LHI_record_queue.h"
#include "control_sgtl5000.h"
#include <Audio.h>  //this also includes SD.h from lines 89 & 90
#include <Wire.h>
#include <SPI.h>
#include "SdFat.h"
#include <Snooze.h>  //using https://github.com/duff2013/Snooze; uncomment line 62 #define USE_HIBERNATE
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);
#define BOTTOM 55

// set this to the hardware serial port you wish to use
#define HWSERIAL Serial1

#define NREC 32 // increase disk buffer to speed up disk access

static uint32_t myID[2];

unsigned long baud = 115200;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

#define MODE_NORMAL 0
#define MODE_DIEL 1

// GUItool: begin automatically generated code
AudioInputI2S            i2s2;           //xy=105,63
LHIRecordQueue           queue1;         //xy=281,63
LHIRecordQueue           queue2;         //xy=281,63
AudioConnection          patchCord1(i2s2, 0, queue1, 0);
AudioConnection          patchCord2(i2s2, 1, queue2, 0);
AudioControlSGTL5000     sgtl5000_1;     //xy=265,212
// GUItool: end automatically generated code

const int myInput = AUDIO_INPUT_LINEIN;
int gainSetting = 4; //default gain setting; can be overridden in setup file


// Pin Assignments
const int UP = 4;
const int DOWN = 3;
const int SELECT = 8;
const int hydroPowPin = 2;
const int vSense = 21; 
#define LEDGREEN 17

// microSD chip select pins
#define CS1 10
#define SDPOW1 20
#define SGTL_EN 6
#define DS3231_INT_PIN 16
int chipSelect[4];
int sdPowSelect[4];
uint32_t freeMB[4];
uint32_t filesPerCard[4];
float fileMB;  // file size including a cluster, set by calcDeployment()
volatile int currentCard = 0;
boolean newCard = 0;


// Pins used by audio shield
// https://www.pjrc.com/store/teensy3_audio.html
// MEMCS 6
// MOSI 7
// BCLK 9
// SDCS 10
// MCLK 11
// MISO 12
// RX 13
// SCLK 14
// VOL 15
// SDA 18
// SCL 19
// TX 22
// LRCLK 23

// Remember which mode we're doing
int mode = 0;  // 0=stopped, 1=recording, 2=playing
time_t startTime;
time_t stopTime;
time_t t;

byte startHour, startMinute, endHour, endMinute; //used in Diel mode
long dielRecSeconds;

int32_t lhi_fsamps[9] = {8000, 16000, 32000, 44100, 48000, 96000, 200000, 250000, 300000};
#define I_SAMP 8   // 0 is 8 kHz; 1 is 16 kHz; 2 is 32 kHz; 3 is 44.1 kHz; 4 is 48 kHz; 5 is 96 kHz; 6 is 192 kHz

float audio_srate = lhi_fsamps[I_SAMP];
int isf = 4; // default to 48 kHz
float gainDb;
int recMode = MODE_NORMAL;
unsigned long rec_dur = 60;
unsigned long rec_int = 60;

int buf_count;
unsigned long nbufs_per_file;
boolean settingsChanged = 0;

volatile uint16_t delayDays = 0;

unsigned long file_count;
char filename[100];
char dirname[20];
int folderMonth;

// The internal RTC alarm wakes the logger. The DS3231 is the time reference only, so a
// faulty DS3231 cannot stop the logger waking.
SnoozeAlarm rtcAlarm;    // Kinetis internal RTC alarm
SnoozeTimer lptmr;       // LPTMR on the 1 kHz LPO; backup wake at least once a minute
SnoozeAudio snooze_audio;
SnoozeBlock config_internal(snooze_audio, rtcAlarm, lptmr);
SnoozeBlock config_lptmr(snooze_audio, lptmr);
SnoozeBlock config_off(snooze_audio);  // no wake source: sleeps until power is cycled
#define LPTMR_WAKE 36  // Snooze wake source for the LPTMR

// Clock sources, best first. The DS3231 is used while it works. If it stops answering
// or stops counting, time comes from the Kinetis internal RTC, and if that is not
// running either, from millis() plus a count of LPTMR sleep periods.
#define CLOCK_DS3231 0
#define CLOCK_INTERNAL 1
#define CLOCK_LPTMR 2
int clockSource = CLOCK_DS3231;
boolean internalRtcOK = false;
boolean rtcFail = false;  // DS3231 has failed and a backup clock is in use
boolean dsOscStopped = false;  // DS3231 oscillator stop flag was set at power-up
float lpoHz = 1000.0;     // measured LPO frequency
time_t softTime = 0;      // millis()-based clock used with the LPTMR
uint32_t softMillis = 0;
#define DEFAULT_TIME 1793548800  // 2026-11-01 16:00:00, used when no clock has a valid time

byte voltCutoff = 33;  // stop recording below this battery voltage, in tenths of a volt; 0 = off
int lowVoltCount = 0;

// EEPROM bytes read by the setup page (tools/logger-setup.html); cleared from the page
#define EE_SD_RESTARTS 25    // count of resetFunc() restarts
#define EE_LOW_BATT_STOP 26  // 1 if recording stopped on low battery
#define EE_CARD_FULL_STOP 27 // 1 if recording stopped with the card full
#define EE_DEPLOYED 28       // 1 while a deployment is running; cleared at power-up
boolean resumeDeployment = false;  // restarted by resetFunc() during a deployment

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 3
const uint8_t SD_CS_PIN = 10;
#define SPI_CLOCK SD_SCK_MHZ(50)

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

// Set PRE_ALLOCATE true to pre-allocate file clusters.
const bool PRE_ALLOCATE = true;

// Set SKIP_FIRST_LATENCY true if the first read/write to the SD can
// be avoid by writing a file header or reading the first record.
const bool SKIP_FIRST_LATENCY = true;

#if SD_FAT_TYPE == 0
SdFat sd;
File file;
#elif SD_FAT_TYPE == 1
SdFat32 sd;
File32 file;
#elif SD_FAT_TYPE == 2
SdExFat sd;
ExFile file;
#elif SD_FAT_TYPE == 3
SdFs sd;
FsFile file;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

typedef struct {
    char    rId[4];
    unsigned int rLen;
    char    wId[4];
    char    fId[4];
    unsigned int    fLen;
    unsigned short nFormatTag;
    unsigned short nChannels;
    unsigned int nSamplesPerSec;
    unsigned int nAvgBytesPerSec;
    unsigned short nBlockAlign;
    unsigned short  nBitsPerSamples;
    char    dId[4];
    unsigned int    dLen;
} HdrStruct;

HdrStruct wav_hdr;

unsigned char prev_dtr = 0;

struct TIME_HEAD
{
  byte  sec;  
  byte  minute;  
  byte  hour;  
  byte  day;  
  byte  month;  
    byte NU[3];
  int16_t year;  
  int16_t tzOffset; //offset from GMT 
};

void setup() {
  read_myID();

  // A software reset (resetFunc) during a deployment goes straight back to recording,
  // without the menu wait or delay days. Power-up, low voltage or any other reset
  // starts fresh with the menu.
  boolean softwareReset = (RCM_SRS1 & 0x04) && !(RCM_SRS0 & 0x82);  // SW set; POR and LVD clear
  resumeDeployment = softwareReset & (EEPROM.read(EE_DEPLOYED) == 1);
  if(!resumeDeployment & (EEPROM.read(EE_DEPLOYED) != 0)) EEPROM.write(EE_DEPLOYED, 0);

  chipSelect[0] = CS1;
  sdPowSelect[0] = SDPOW1;
  
  Serial.begin(baud);
  delay(500);

  pinMode(LEDGREEN, OUTPUT);
  digitalWrite(LEDGREEN, HIGH);
  
  Wire.begin();
  pinMode(DS3231_INT_PIN, INPUT_PULLUP); // DS3231 INT, not used to wake; read for diagnostics
  pinMode(vSense, INPUT);
  analogReference(INTERNAL);
  
  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, HIGH);

  pinMode(SDPOW1, OUTPUT);
  pinMode(SGTL_EN, OUTPUT);
  digitalWrite(SDPOW1, LOW); // start cardsswitched off in case of reset
  digitalWrite(SGTL_EN, HIGH);
  
  //setup display and controls
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(SELECT, INPUT_PULLUP);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
  delay(140);
  cDisplay();
  display.println("Loggerhead");
  display.setTextSize(1);
  display.print("Version ");
  display.println(codeVersion);
  display.display();
  delay(1000); // leave splash up long enough to read the version

  // Audio connections require memory, and the record queue
  // uses this memory to buffer incoming audio.
  // initialize now to estimate DC offset during setup
  AudioMemory(MQ+10);
  
  audio_srate = lhi_fsamps[isf];
//WMXZ  audioIntervalSec = 256.0 / audio_srate; //buffer interval in seconds

  clockInit();  // check the DS3231 and fall back to the internal RTC if it has failed

  AudioInit(isf); // load current gain setting
  manualSettings();
  EEPROM.write(EE_DEPLOYED, 1);  // deployment starts; no write if already set
  audio_srate = lhi_fsamps[isf];
  AudioInit(isf); // set with new settings

  cDisplay();

  if(rec_int >= 60) roundSeconds = 60;
  if(rec_int >= 300) roundSeconds = 300;
  
  t = getTeensy3Time(1);  // sync teensy rtc to DS3231
  startTime = t;
  startTime -= startTime % roundSeconds;  
  startTime += roundSeconds; //move forward
  if(!resumeDeployment) startTime += delayDays * 86400;  // the delay was already served before a restart
  stopTime = startTime + rec_dur;  // this will be set on start of recording


  // delay start
  if((delayDays > 0) & !resumeDeployment){
    // power down SD, SGTL
    digitalWrite(hydroPowPin, LOW);
    audioSleep();
    i2sPinsLow();  // the codec power is cut in each sleep; AudioInit() below restores the pins
    digitalWrite(SDPOW1, LOW);
    
    // sleep walk (~3.7 mA current)
    long time_to_first_rec = startTime - t;
    Serial.print("Time to first record ");
    Serial.println(time_to_first_rec);
    for(int x = 0; x<30; x++){
      cDisplay();
      display.println("Sleep");
      t = getTeensy3Time(1);
      display.setTextSize(1);
      display.println("Start");
      displayClock(startTime, 26);
      displayClock(t, BOTTOM);
      display.display();
      delay(1000);
    }

    while(time_to_first_rec > 60){
      display.ssd1306_command(SSD1306_DISPLAYOFF);
      digitalWrite(LEDGREEN, HIGH);
      t = getTeensy3Time(0);
      delay(10);
      digitalWrite(LEDGREEN, LOW);
      sleepUntil(t + 20, 1); // sleep 20 seconds with SGTL5000 off
      t = getTeensy3Time(1);
      time_to_first_rec = startTime - t;
    }
    digitalWrite(hydroPowPin, HIGH);
    digitalWrite(SDPOW1, HIGH);
    digitalWrite(SGTL_EN, HIGH);
    delay(1000);
    // Initialize the SD card
    SPI.setMOSI(7);
    SPI.setSCK(14);
    SPI.setMISO(12);
    sd.begin(chipSelect[0]);
    AudioInit(isf);
  }


  if (recMode==MODE_DIEL) setDielTime();  // adjust start time to diel mode
  
  nbufs_per_file = (long) (ceil(((rec_dur * audio_srate / 256.0) / (float) NREC)) * (float) NREC) * NCHAN;

  Serial.print("rec dur ");
  Serial.println(rec_dur);
  Serial.print("rec int ");
  Serial.println(rec_int);
  Serial.print("Current Time: ");
  printTime(t);
  Serial.print("Start Time: ");
  printTime(startTime);
  
  // Sleep here if won't start for 60 s
  long time_to_first_rec = startTime - t;
  Serial.print("Time to first record ");
  Serial.println(time_to_first_rec);
  
  mode = 0;

  // create first folder to hold data
  folderMonth = -1;  //set to -1 so when first file made will create directory
  checkSD();
}

//
// MAIN LOOP
//

int recLoopCount;  //for debugging when does not start record
  
void loop() {
  // Standby mode
  if(mode == 0)
  {
      t = getTeensy3Time(0);
      cDisplay();
      display.println("Next Start");
      display.setTextSize(1);
      display.setCursor(0, 18);
      display.print("Card:");
      display.print(currentCard + 1);
      display.print(" ");
      display.print(filesPerCard[currentCard]);
      displayClock(startTime, 40);
      displayClock(t, BOTTOM);
      display.display();

      if(t >= startTime){      // time to start?
        if(noDC==0) {
          audio_freeze_adc_hp(); // this will lower the DC offset voltage, and reduce noise
        }
        if(noDC==2){
          audio_bypass_adc_hp();
         }
        Serial.println("Record Start.");
        if(sleepDiags) printWakeLog();  // wakes from the sleep that just ended

        // set current stop time and calculate next startTime
        if(recMode==MODE_NORMAL){
          stopTime = startTime + rec_dur;
          startTime = stopTime + rec_int;
        }
        
        if(recMode==MODE_DIEL){
          if (rec_int==0){
             stopTime = startTime + dielRecSeconds;
             startTime = startTime + (24*3600); // increment startTime by 1 day
             Serial.print("New diel stop:");
             printTime(stopTime);
          }
          else{
            stopTime = startTime + rec_dur;
            startTime = stopTime + rec_int;
            setDielTime(); // make sure new start is in diel window
          }
        }
        Serial.print("Current Time: ");
        printTime(getTeensy3Time(0));
        Serial.print("Stop Time: ");
        printTime(stopTime);
        Serial.print("Next Start:");
        printTime(startTime);

        mode = 1;
        display.ssd1306_command(SSD1306_DISPLAYOFF); // turn off display during recording
        startRecording();
      }
  }


  // Record mode
  if (mode == 1) {
    continueRecording();  // download data  
    if(sleepDiags) checkAudioFlow();

    if(digitalRead(UP)==0 & digitalRead(DOWN)==0){
      // stop recording
      queue1.end();
      queue2.end();
      // update wav file header
      wav_hdr.rLen = 36 + buf_count * 256 * 2;
      wav_hdr.dLen = buf_count * 256 * 2;
      file.seek(0);
      file.write((uint8_t *)&wav_hdr, 44);
      file.close();

      display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
      delay(100);
      cDisplay();
      display.println("Stopped");
      display.setTextSize(1);
      display.println("Safe to turn off");
      display.display();
      EEPROM.write(EE_DEPLOYED, 0);  // deployment stopped by hand
      while(1);
    }
    
    if(buf_count >= nbufs_per_file){       // time to stop?
      if(((rec_int == 0) & (recMode==MODE_NORMAL)) | ((rec_int == 0) & (recMode==MODE_DIEL) & (getTeensy3Time(0)<stopTime))){
        file.close();
        checkSD();
        checkVoltage();
        FileInit();  // make a new file
        buf_count = 0;
        if(printDiags) {
          Serial.print("Audio Mem: ");
          Serial.println(AudioMemoryUsageMax());
        }
      }
      else{
        stopRecording();
        checkSD();
        checkVoltage();
        long sleepSeconds = startTime - getTeensy3Time(0) - wakeahead;
        if (sleepSeconds < 0) sleepSeconds = 0;

        if(sleepSeconds >= 10){
            time_t wakeTime = getTeensy3Time(0) + sleepSeconds;

            digitalWrite(hydroPowPin, LOW); //hydrophone off
            audioSleep();  // codec down, then I2S stopped; AudioInit() restarts the I2S, then the codec

            if(printDiags){
              Serial.print("Sleep seconds: ");
              Serial.println(sleepSeconds);
              Serial.flush();
            }
            delay(100);

            sleepUntil(wakeTime, 0);

            // Waking up
            digitalWrite(hydroPowPin, HIGH); // hydrophone on
            delay(300);  // give time for Serial to reconnect to USB
            AudioInit(isf);
         }

        display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  //initialize display
        mode = 0;  // standby mode
      }
    }
  }
  asm("wfi"); // reduce power between interrupts
}

void startRecording() {
  if (printDiags)  Serial.println("startRecording");
  FileInit();
  buf_count = 0;
  queue1.begin();
  if(NCHAN==2) queue2.begin();
  if(sleepDiags) diagRecordStart();
  if (printDiags)  Serial.println("Queue Begin");
}


byte buffer[NREC*512];
void continueRecording() {
  if(NCHAN==1){
    if (queue1.available() >= NREC*2) {
      // Fetch 2 blocks (or multiples) from the audio library and copy
      // into a 512 byte buffer.  micro SD disk access
      // is most efficient when full (or multiple of) 512 byte sector size
      // writes are used.
      digitalWrite(LEDGREEN, HIGH);
      for(int ii=0;ii<NREC;ii++)
      { byte *ptr = buffer+ii*512;
        memcpy(ptr, queue1.readBuffer(), 256);
        queue1.freeBuffer();
        memcpy(ptr+256, queue1.readBuffer(), 256);
        queue1.freeBuffer();
      }
      digitalWrite(LEDGREEN, LOW);
      if(file.write(buffer, NREC*512)==-1) resetFunc(); //audio to .wav file
      buf_count += NREC;
    }
  }

  if(NCHAN==2){
    if (queue1.available() >= NREC) {
      // one buffer is 512 bytes = 256 samples
      // readBuffer returns an int16 * that contains 256 bytes
      digitalWrite(LEDGREEN, HIGH);
      for(int ii=0;ii<NREC;ii++){ 
        byte *ptr = buffer+(ii*512);
        mxLR(ptr, queue1.readBuffer(), queue2.readBuffer()); // interleave 256 points from each
        queue1.freeBuffer(); 
        queue2.freeBuffer();  // free buffer
      }
      digitalWrite(LEDGREEN, LOW);
      if(file.write(buffer, NREC*512)==-1) resetFunc(); //audio to .wav file
      
      buf_count += NREC;
//      if(printDiags){
//        Serial.print(buf_count);
//        Serial.print(" ");
//        Serial.println((int16_t) buffer[1]<<8 | buffer[0]);
//      }
    }
  }

}

inline void mxLR(byte *dst, const int16_t *srcL, const int16_t *srcR){
    byte cnt = 128;
    int16_t *d = (int16_t *)dst;
    const int16_t *l = srcL;
    const int16_t *r = srcR;

    while (cnt--)
    {
      *(d++) = *l++;
      *(d++) = *r++;
    }
 }
  
void stopRecording() {
  if (printDiags) Serial.println("stopRecording");
  int maxblocks = AudioMemoryUsageMax();
  if (printDiags) Serial.print("Audio Memory Max");
  if (printDiags) Serial.println(maxblocks);
  byte buffer[512];
  queue1.end();
  if(NCHAN==2) queue2.end();
  AudioMemoryUsageMaxReset();
  file.close();
  delay(100);
}


void FileInit()
{
   t = getTeensy3Time(1); // this will also sync teensy clock used to wake up to DS3231

   // use folders for duty cycle record because short files could exceed max files per directory
   if (folderMonth != month(t) & (rec_int > 0)){
    if(printDiags) Serial.println("New Folder");
    folderMonth = month(t);
    sprintf(dirname, "/%04d-%02d", year(t), folderMonth);

    sd.mkdir(dirname);
    sd.chdir(dirname);
   }
   pinMode(vSense, INPUT);  // get ready to read voltage
   float voltage = readVoltage();

  #if USE_SDFS==1
    FsDateTime::callback = file_date_time;
  #else
    SdFile::dateTimeCallback(file_date_time);
  #endif

   // open file 
   sprintf(filename,"%04d%02d%02dT%02d%02d%02d_%lu%lu_%2.1fdB_%2.1fV_ver%s.wav", year(t), month(t), day(t), hour(t), minute(t), second(t), myID[0], myID[1], gainDb, voltage, codeVersion);  //filename is DDHHMMSS
   Serial.println(filename);
   while (!file.open(filename, O_WRITE | O_CREAT | O_EXCL)){
    file_count += 1;
    sprintf(filename,"F%06d.wav",file_count); //if can't open just use count
    sd.chdir(dirname);
    file.open(filename, O_WRITE | O_CREAT | O_EXCL);
    Serial.println(filename);
    delay(10);
    if(file_count>20) {
      resetFunc();
      delay(1000);
    }
   }

    //intialize stereo .wav file header
    sprintf(wav_hdr.rId,"RIFF");
    wav_hdr.rLen=36;
    sprintf(wav_hdr.wId,"WAVE");
    sprintf(wav_hdr.fId,"fmt ");
    wav_hdr.fLen=0x10;
    wav_hdr.nFormatTag=1;
    wav_hdr.nChannels = NCHAN;
    wav_hdr.nSamplesPerSec=audio_srate;
    wav_hdr.nAvgBytesPerSec = audio_srate * 2 * NCHAN;
    wav_hdr.nBlockAlign = 2 * NCHAN;
    wav_hdr.nBitsPerSamples=16;
    sprintf(wav_hdr.dId,"data");
    wav_hdr.dLen = nbufs_per_file * 256 * 2;
    wav_hdr.rLen = 36 + wav_hdr.dLen;

    file.write((uint8_t *)&wav_hdr, 44);

    Serial.print("Buffers: ");
    Serial.println(nbufs_per_file);
}

//This function returns the date and time for SD card file access and modify time. One needs to call in setup() to register this callback function: SdFile::dateTimeCallback(file_date_time);
void file_date_time(uint16_t* date, uint16_t* time) 
{
  t = getTeensy3Time(1);
  #if USE_SDFS==1
    *date=FS_DATE(year(t),month(t),day(t));
    *time=FS_TIME(hour(t),minute(t),second(t));
  #else
    *date=FAT_DATE(year(t),month(t),day(t));
    *time=FAT_TIME(hour(t),minute(t),second(t));
  #endif
}

void AudioInit(int ifs){
 // Instead of using audio library enable; do custom so only power up what is needed in sgtl5000_LHI
  I2S_modification(lhi_fsamps[ifs], 16);
  Wire.begin();
  audio_enable(ifs);
}

void calcGain(){
    switch(gainSetting){
    case 0: gainDb = -20 * log10(3.12 / 2.0); break;
    case 1: gainDb = -20 * log10(2.63 / 2.0); break;
    case 2: gainDb = -20 * log10(2.22 / 2.0); break;
    case 3: gainDb = -20 * log10(1.87 / 2.0); break;
    case 4: gainDb = -20 * log10(1.58 / 2.0); break;
    case 5: gainDb = -20 * log10(1.33 / 2.0); break;
    case 6: gainDb = -20 * log10(1.11 / 2.0); break;
    case 7: gainDb = -20 * log10(0.94 / 2.0); break;
    case 8: gainDb = -20 * log10(0.79 / 2.0); break;
    case 9: gainDb = -20 * log10(0.67 / 2.0); break;
    case 10: gainDb = -20 * log10(0.56 / 2.0); break;
    case 11: gainDb = -20 * log10(0.48 / 2.0); break;
    case 12: gainDb = -20 * log10(0.40 / 2.0); break;
    case 13: gainDb = -20 * log10(0.34 / 2.0); break;
    case 14: gainDb = -20 * log10(0.29 / 2.0); break;
    case 15: gainDb = -20 * log10(0.24 / 2.0); break;
  }
}

int dsReadFails = 0;
time_t lastDsTime = 0, lastRtcTime = 0;
uint32_t lastDsChange = 0, lastRtcChange = 0;
time_t lastDsChangeRtc = 0;  // internal RTC time when the DS3231 reading last changed

// True while a clock keeps counting; false once its reading has not changed for 5 s
boolean clockCounting(time_t now, time_t *last, uint32_t *lastChange){
  if(now != *last){
    *last = now;
    *lastChange = millis();
  }
  return (millis() - *lastChange < 5000);
}

// Returns the time from the best working clock and switches to a backup clock
// when the DS3231 stops answering (3 failed reads in a row) or stops counting.
// The DS3231 counts as stuck when its time has not changed for 5 s by millis() (awake)
// or by the internal RTC, which also counts during sleep.
time_t getTeensy3Time(boolean syncTeensy)
{
  if(clockSource == CLOCK_DS3231){
    if(readRTC()){
      dsReadFails = 0;
      boolean changed = (t != lastDsTime);
      if(changed){
        lastDsTime = t;
        lastDsChange = millis();
        lastDsChangeRtc = Teensy3Clock.get();
      }
      boolean stuck = !changed && ((millis() - lastDsChange >= 5000) ||
                                   (internalRtcOK && (long) (Teensy3Clock.get() - lastDsChangeRtc) >= 5));
      if(!stuck){
        setSoftTime(t);
        if(syncTeensy){
          setTeensyTime(hour(t),minute(t),second(t),day(t),month(t),year(t)); // sync from DS3231 to Teensy time
          if(changed) lastDsChangeRtc = Teensy3Clock.get();  // measure from the synced time
        }
        return t;
      }
      useBackupClock();  // DS3231 answers but its time is stuck
    }
    else if(++dsReadFails >= 3) useBackupClock();
  }

  if(internalRtcOK){
    t = Teensy3Clock.get();
    if(clockCounting(t, &lastRtcTime, &lastRtcChange)){
      setSoftTime(t);
      return t;
    }
    internalRtcOK = false;  // internal RTC stopped too
    if(clockSource == CLOCK_INTERNAL) clockSource = CLOCK_LPTMR;
  }

  t = softNow();
  return t;
}

void resetFunc(void){
  byte sdRestarts = EEPROM.read(EE_SD_RESTARTS);
  if(sdRestarts < 254) EEPROM.write(EE_SD_RESTARTS, sdRestarts + 1);
  if(mode == 1) file.close();  // close file if open
  // let the card finish any operation in progress before its power is cut
  if(sd.card()){
    uint32_t busyStart = millis();
    while(sd.card()->isBusy() && (millis() - busyStart < 2000)) delay(10);
  }
  // MISO, MOSI, SCLK LOW
  digitalWrite(7, LOW);
  digitalWrite(12, LOW);
  digitalWrite(14, LOW);
  pinMode(7, INPUT_DISABLE);
  pinMode(12, INPUT_DISABLE);
  pinMode(14, INPUT_DISABLE);
  
  //cycle power on SD cards (in case there are cards in other slots)
  for(int n = 0; n<1; n++){
    digitalWrite(sdPowSelect[n], LOW);
    digitalWrite(chipSelect[n], LOW);
    pinMode(chipSelect[n], INPUT_DISABLE);
  }
  delay(2000);
  for(int n = 0; n<1; n++){
    digitalWrite(sdPowSelect[n], HIGH);
  }
  delay(2000);
  for(int n = 0; n<1; n++){
    digitalWrite(sdPowSelect[n], LOW);
  }
  delay(2000);
  CPU_RESTART
}

// Switch the card off and on again. The SPI pins are released and CS held low
// while it is off so nothing powers the card through them.
void sdPowerCycle(int n){
  pinMode(7, INPUT_DISABLE);   // MOSI
  pinMode(12, INPUT_DISABLE);  // MISO
  pinMode(14, INPUT_DISABLE);  // SCK
  pinMode(chipSelect[n], OUTPUT);
  digitalWrite(chipSelect[n], LOW);
  digitalWrite(sdPowSelect[n], LOW);
  delay(2000);
  digitalWrite(sdPowSelect[n], HIGH);
  delay(1000);  // sd.begin() reconnects the SPI pins
}

void read_myID() {
//  myID[0] = SIM_UIDH;
  myID[0] = SIM_UIDMH;
  myID[1] = SIM_UIDML;
//  myID[3] = SIM_UIDL;
}


float readVoltage(){
   float  voltage = 0.0;
   // Serial.println(analogRead(vSense));
   for(int n = 0; n<1000; n++){
    voltage += (float) analogRead(vSense);
   }
   // Serial.println(voltage / 1000.0);
   voltage = (voltage / 173868.0);   // experimentally determined....it varies ~5% each time powered on
   return voltage;
}

void checkSD(){
  if (filesPerCard[currentCard] == 0){
    // The power-up estimate says the card is full. Count the free space before stopping.
    int32_t volFree = sd.vol()->freeClusterCount();
    if(volFree <= 0) resetFunc();  // count failed (a full card still has the 200 MB margin): restart and retry the card
    float freeSpace = 0.000512 * volFree * sd.vol()->blocksPerCluster();
    if (freeSpace < 200) freeMB[currentCard] = 0;
    else freeMB[currentCard] = freeSpace - 200; // same 200 MB margin as at power-up
    if (freeMB[currentCard] > fileMB) filesPerCard[currentCard] = (uint32_t) floor(freeMB[currentCard] / fileMB);
    Serial.print("Files left after recount: ");
    Serial.println(filesPerCard[currentCard]);
    if (filesPerCard[currentCard] == 0) stopForGood(EE_CARD_FULL_STOP, "Card Full", "");  // all the files that fit are recorded
  }
  filesPerCard[currentCard] -= 1;

  if(printDiags){
    Serial.print("Files per card: ");
    Serial.println(filesPerCard[currentCard]);
  }
}

// Called between files. Stops for good once the battery has read below the cutoff
// at 3 file changes in a row, so a single low reading does not end a deployment.
void checkVoltage(){
  if(voltCutoff == 0) return;  // cutoff turned off
  pinMode(vSense, INPUT);
  float voltage = readVoltage();
  if(voltage < voltCutoff / 10.0) lowVoltCount++;
  else lowVoltCount = 0;
  if(lowVoltCount >= 3){
    char detail[12];
    sprintf(detail, "%.1fV", voltage);
    stopForGood(EE_LOW_BATT_STOP, "Low Batt", detail);
  }
}

// Recording is over (card full or battery low): power everything down and sleep until
// the power is cycled. eeFlag records why, for the setup page. The last file is already closed.
void stopForGood(int eeFlag, const char *title, const char *detail){
  Serial.print(title);
  Serial.print(" stop ");
  Serial.println(detail);
  queue1.end();
  if(NCHAN==2) queue2.end();
  digitalWrite(hydroPowPin, LOW);
  audioSleep();

  // let the card finish any operation in progress, then switch it off
  if(sd.card()){
    uint32_t busyStart = millis();
    while(sd.card()->isBusy() && (millis() - busyStart < 2000)) delay(10);
  }
  pinMode(7, INPUT_DISABLE);   // MOSI
  pinMode(12, INPUT_DISABLE);  // MISO
  pinMode(14, INPUT_DISABLE);  // SCK
  digitalWrite(chipSelect[0], LOW);
  digitalWrite(sdPowSelect[0], LOW);

  display.ssd1306_command(SSD1306_DISPLAYON);
  cDisplay();
  display.println(title);
  display.println("Stopped");
  display.setTextSize(1);
  display.print(detail);
  display.display();
  delay(5000);
  display.ssd1306_command(SSD1306_DISPLAYOFF);

  EEPROM.write(eeFlag, 1);
  EEPROM.write(EE_DEPLOYED, 0);  // deployment is over
  disable_alarm();  // so the DS3231 does not hold INT low
  i2sPinsLow();  // before the codec power is cut
  digitalWrite(SGTL_EN, LOW);
  digitalWrite(LEDGREEN, LOW);
  while(1) Snooze.deepSleep(config_off);
}

void setDielTime(){
  unsigned int startMinutes = (startHour * 60) + (startMinute);
  unsigned int endMinutes = (endHour * 60) + (endMinute );
  unsigned int startTimeMinutes =  (hour(startTime) * 60) + (minute(startTime));
  
  tmElements_t tmStart;
  tmStart.Year = year(startTime) - 1970;
  tmStart.Month = month(startTime);
  tmStart.Day = day(startTime);


  // check if next startTime is between startMinutes and endMinutes
  // e.g. 06:00 - 12:00 or 
  if(startMinutes<endMinutes){
     if ((startTimeMinutes < startMinutes) | (startTimeMinutes > endMinutes)){
       // set startTime to startHour startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       if(startTime < getTeensy3Time(0)) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
     }
     dielRecSeconds = (endMinutes-startMinutes) * 60;
     Serial.print("Diel Rec Seconds");
     Serial.println(dielRecSeconds);
  }
  else{  // e.g. 23:00 - 06:00
    if((startTimeMinutes<startMinutes) & (startTimeMinutes>endMinutes)){
      // set startTime to startHour:startMinute
       tmStart.Hour = startHour;
       tmStart.Minute = startMinute;
       tmStart.Second = 0;
       startTime = makeTime(tmStart);
       if(startTime < getTeensy3Time(0)) startTime += SECS_PER_DAY;  // make sure after current time
       Serial.print("New diel start:");
       printTime(startTime);
    }
    dielRecSeconds = ((1440 - startMinutes) * 60) + (endMinutes * 60);
    Serial.print("Diel Rec Seconds");
    Serial.println(dielRecSeconds);
  }
}
