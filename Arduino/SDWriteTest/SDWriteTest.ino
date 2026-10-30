//
// SD card write test for LS1/LS2 v4 boards
//
// Writes files to the card the way LS1-LS2-v4 records a duty cycle:
// - the same SdFat calls (sd.begin, mkdir/chdir, open, write, close), a 44-byte header
//   followed by 16384-byte writes (NREC * 512), the same file size, names and day folders
// - each write happens when that much audio would have arrived (every 85 ms at 48 kHz stereo)
// - between files, the same steps as the logger: 100 ms after closing (stopRecording), deep
//   sleep, wake 4 s early (wakeahead), 300 ms after waking, then standby until the scheduled
//   start. Sleeps under 10 s are skipped, as on the logger. With sdSleep set (SD Sleep
//   Power off, the logger's default) the card is switched off for sleeps of 2 min or more.
// - continuous mode (rec_int = 0): close and open the next file while audio keeps arriving,
//   so the open time counts against the audio buffer
// - card errors are handled with copies of the logger's card functions: a failed open tries
//   20 numbered names, a failed write or open restarts the processor with the card power
//   cycled (resetFunc), the card is started again as manualSettings() does, and the test
//   resumes. After 5 failed restarts in a row it waits an hour with the card off.
// - the logger's 60 s hardware watchdog. The step the test is in is kept in a register that
//   survives the restart, and the watchdog interrupt saves the program address just before
//   the reset, so a hang shows where it was (display, serial and WRTEST.TXT).
// Audio itself is not recorded; the buffer is a stand-in. The display stays on with the
// results and is only updated between files, when the logger also uses it.
//
// Every write is timed. A line per file goes to USB serial, and a summary is kept in
// WRTEST.TXT in the card's root folder.
//
// Display line "W O C R WD": write, open and close errors, card restarts, watchdog restarts
// and the step the last watchdog restart was in.
//
// The run's counts are kept in EEPROM from byte 1024 (the logger's settings and counts are in
// bytes 0-37 and are not changed), so a restart resumes the run. Powering up starts a new run.
//
// Press UP+DOWN together to stop, as on the logger: the current file is closed with its
// header updated, the results are saved, and the display says when it is safe to turn off.
// During deep sleep the buttons are checked when the board wakes, at least once a minute.
//
// Filling a 1 TB card this way takes as long as a deployment. To reach the nearly-full state
// sooner, set fastFillGB: that much is written back-to-back first, then the test runs as the
// logger.
//
// Prepare the card the way you want to test it (Windows quick format, SD Card Formatter
// overwrite, or the SdFat SdFormatter example), then power up the board.
// Compile as for the logger: Teensy 3.2, 96 MHz. The test uses its own copy of SdFat in
// src/SdFat (Teensyduino's SdFat 2.1.2, see SOURCE.txt), not the SdFat-beta library the logger
// uses; the write timeout is the same. Keep the build's .elf to look up the program address
// after a hang.
//

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include "src/SdFat/SdFat.h"  // this sketch's own copy (src/SdFat/SOURCE.txt), not the SdFat-beta library
#include <Snooze.h>
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*********** Settings: match the deployment being tested ***********
const unsigned long rec_dur = 300;     // record duration (s)
const unsigned long rec_int = 10;      // sleep between recordings (s); 0 = continuous
const float audio_srate = 48000;       // sample rate (Hz)
const int NCHAN = 2;                   // channels
const byte sdSleep = 1;                // 1: card off for sleeps of 2 min or more (SD Sleep Power off)
const float fastFillGB = 0;            // write this many GB back-to-back first, then run as the logger
const float testGB = 0;                // stop after this many GB in total; 0 = fill the card
//*******************************************************************

#define NREC 32       // same as LS1-LS2-v4: each write is NREC * 512 = 16384 bytes
#define SD_SLEEP_SECONDS 120  // same as LS1-LS2-v4
int wakeahead = 4;    // same as LS1-LS2-v4
char codeVersion[5] = "4.50";  // only used in file names, as the logger does
float gainDb = 2.0;            // only used in file names
float voltage = 4.0;           // only used in file names

// Audio the logger can buffer while a write is in progress, per channel: AudioMemory(MQ+10)
// = 110 blocks of 128 samples, shared by the two queues in stereo. An estimate, used to
// count how much audio a slow write would have dropped.
const float bufferSamples = (NCHAN == 2 ? 55 : 100) * 128;

// Pins, as LS1-LS2-v4
#define CS1 10
#define SDPOW1 20
#define SGTL_EN 6
#define LEDGREEN 17
const int hydroPowPin = 2;
const int UP = 4;
const int DOWN = 3;

#define CPU_RESTART_ADDR (uint32_t *)0xE000ED0C
#define CPU_RESTART_VAL 0x5FA0004
#define CPU_RESTART (*CPU_RESTART_ADDR = CPU_RESTART_VAL);

int chipSelect[4];
int sdPowSelect[4];
boolean cardIsOn = false;
unsigned long file_count = 0;  // numbered names after a failed open, as the logger

#define OLED_RESET -1
Adafruit_SSD1306 display(OLED_RESET);

SnoozeTimer lptmr;
SnoozeBlock config_sleep(lptmr);

SdFs sd;
FsFile file;

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
byte buffer[NREC*512];
char filename[100];
char dirname[20];
long folderDate = -1;  // YYYYMMDD of the current folder
uint32_t myID[2];
unsigned long nbufs_per_file;
uint32_t writesPerFile;
uint32_t fileBytes;
const float samplesPerWrite = NREC * 512 / (2 * NCHAN);  // per channel

// Card
char fsName[8];
uint32_t clusterKB;
float cardGB;
cid_t cid;

// Schedule, in millis(); Snooze adds the LPTMR sleep time to millis()
uint32_t nextStartMs = 0;
boolean scheduled = false;

// Audio arriving while recording
boolean audioRunning = false;
uint32_t audioLastUs;
double audioProduced, audioConsumed;  // samples per channel

const uint16_t slowMs[] = {50, 100, 250, 600, 1000};  // 600 ms is SdFat-beta's write timeout
#define NSLOW 5
#define NSEG 20  // results by twentieths of the test, to see whether writes slow down as the card fills

// Steps, kept in a register as the test goes, so a hang can be traced after the restart
enum { STEP_NONE, STEP_CARD_START, STEP_FREE_COUNT, STEP_FOLDER, STEP_OPEN, STEP_EXISTS,
  STEP_HEADER, STEP_WRITE, STEP_CLOSE, STEP_RESULTS, STEP_SAVE, STEP_DISPLAY, STEP_SLEEP,
  STEP_CARD_OFF, STEP_RESTART, STEP_WAIT, NSTEP };
const char *stepName[NSTEP] = {"none", "start", "free", "folder", "open", "exists", "header",
  "write", "close", "result", "save", "disp", "sleep", "cardoff", "restart", "wait"};

// System register file words: kept through watchdog and software restarts, not through a
// power-up. Word 7 is the card-failure count, as the logger.
#define RFSYS_REG3 (*(volatile uint32_t *)0x4004100C)  // LR at the watchdog interrupt
#define RFSYS_REG4 (*(volatile uint32_t *)0x40041010)  // PC at the watchdog interrupt
#define RFSYS_REG5 (*(volatile uint32_t *)0x40041014)  // write number in the file
#define RFSYS_REG6 (*(volatile uint32_t *)0x40041018)  // marker, step resetFunc() was called in, step
#define RFSYS_REG7 (*(volatile uint32_t *)0x4004101C)
#define STEP_MARK 0x57540000
#define PC_SAVED 0x80

// One restart: what caused it and where the test was
#define NLOG 8
typedef struct {
  uint8_t cause;    // 1 watchdog, 2 card error (resetFunc), 3 other software restart
  uint8_t step;
  uint8_t pcSaved;  // 0: the watchdog interrupt did not run (interrupts off or in a fault)
  uint8_t spare;
  uint32_t write;   // write number in the file, for the write step
  uint32_t pc, lr;
  uint32_t file;    // file number being written
  float gb;
} RestartRec;
const char *causeName[] = {"", "watchdog", "card error", "software"};

// The run, kept in EEPROM so a restart resumes it
#define STATE_ADDR 1024
#define STATE_RUNNING 0x57525431
#define STATE_DONE 0x57525430
struct {
  uint32_t magic;
  time_t t;  // simulated clock for names, folders and timestamps: advances rec_dur + rec_int per file
  uint32_t filesWritten, filesToWrite;
  uint64_t bytesWritten;
  float targetGB;
  uint32_t elapsedSec;
  uint32_t writeCount, slowCount[NSLOW], maxWriteUs;
  float maxWriteGB;
  uint32_t maxOpenMs, maxCloseMs;
  uint32_t writeErrors, openErrors, closeErrors;
  uint32_t sdRestarts, wdogResets, otherResets;
  uint8_t lastWdogStep;
  uint32_t skippedNames;  // file names left by an earlier run or by a restart
  uint32_t sleeps, cardSleeps;
  double droppedSamples;
  uint32_t dropWrites;
  uint16_t segMaxMs[NSEG];
  uint32_t segSlow[NSEG];  // writes over 100 ms
  float segMBps[NSEG];
  int seg;
  uint32_t nRestarts;
  RestartRec restartLog[NLOG];  // the last NLOG
} st;
static_assert(STATE_ADDR + sizeof(st) <= 2048, "test state does not fit in the EEPROM");

uint32_t startMillis, elapsedBase;
uint32_t segStartMillis;
uint64_t segStartBytes = 0;
boolean stopped = false;   // UP+DOWN pressed
boolean finished = false;

void setup() {
  myID[0] = SIM_UIDMH;
  myID[1] = SIM_UIDML;
  chipSelect[0] = CS1;
  sdPowSelect[0] = SDPOW1;

  // As the logger: a software or watchdog restart resumes the run; a power-up starts a new one
  boolean wdogReset = RCM_SRS0 & 0x20;
  boolean restarted = ((RCM_SRS1 & 0x04) || wdogReset) && !(RCM_SRS0 & 0x82);  // SW or WDOG; not POR or LVD
#ifdef SIM_SCGC5_REGFILE
  SIM_SCGC5 |= SIM_SCGC5_REGFILE;
#endif
  uint32_t hang = RFSYS_REG6;
  if((hang & 0xFFFF0000) != STEP_MARK) hang = STEP_MARK;
  uint32_t hangWrite = RFSYS_REG5, hangPc = RFSYS_REG4, hangLr = RFSYS_REG3;
  RFSYS_REG6 = STEP_MARK;
  RFSYS_REG5 = RFSYS_REG4 = RFSYS_REG3 = 0;
  uint8_t failedStep = (hang >> 8) & 0x7F;

  EEPROM.get(STATE_ADDR, st);
  boolean resume = restarted && (st.magic == STATE_RUNNING);
  if(!resume) memset(&st, 0, sizeof(st));
  elapsedBase = st.elapsedSec;
  startMillis = millis();
  if(resume){
    RestartRec &r = st.restartLog[st.nRestarts % NLOG];
    r.cause = wdogReset ? 1 : (failedStep ? 2 : 3);
    r.step = wdogReset ? (hang & 0x7F) : failedStep;
    if(r.step >= NSTEP) r.step = STEP_NONE;
    r.pcSaved = wdogReset && (hang & PC_SAVED);
    r.write = hangWrite;
    r.pc = hangPc;
    r.lr = hangLr;
    r.file = st.filesWritten + 1;
    r.gb = st.bytesWritten / 1e9;
    st.nRestarts++;
    if(wdogReset){
      st.wdogResets++;
      st.lastWdogStep = r.step;
    }
    else if(!failedStep) st.otherResets++;
    saveState();
  }

  Serial.begin(115200);

  // Pins, as the logger
  pinMode(LEDGREEN, OUTPUT);
  pinMode(UP, INPUT_PULLUP);
  pinMode(DOWN, INPUT_PULLUP);
  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, LOW);  // hydrophone not needed
  pinMode(SDPOW1, OUTPUT);
  digitalWrite(SDPOW1, LOW); // start cards switched off in case of reset
  pinMode(SGTL_EN, OUTPUT);

  // Codec power cycle and serial wait, as the logger
  digitalWrite(SGTL_EN, LOW);
  delay(250);
  digitalWrite(SGTL_EN, HIGH);
  delay(250);

  if(resume){
    RestartRec &r = st.restartLog[(st.nRestarts - 1) % NLOG];
    Serial.print("Resumed after restart: ");
    printRestart(Serial, st.nRestarts, r);
    Serial.println();
  }

  Wire.begin();
  setStep(STEP_DISPLAY);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(140);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SD Write Test");
  display.printf("%lus/%lus %.0fkHz %dch\n", rec_dur, rec_int, audio_srate / 1000, NCHAN);
  display.printf("Timeout wr%u cmd%ums\n", SD_WRITE_TIMEOUT, SD_CMD_TIMEOUT);
  if(resume){
    RestartRec &r = st.restartLog[(st.nRestarts - 1) % NLOG];
    display.printf("Resumed: %s\nin %s\n", causeName[r.cause], stepName[r.step]);
  }
  display.display();
  delay(1000);

  watchdogStart();

  // Start the card exactly as manualSettings() in LS1-LS2-v4.
  // After 5 card failures in a row during a deployment, rest for an hour with the card,
  // codec and hydrophone off before trying again, for as long as the batteries last
  if(resume & (cardFailCount() >= 5)){
    Serial.printf("Card failed %d times in a row: retry in 1 hour\n", cardFailCount());
    display.println("Card failed");
    display.println("Retry in 1 hour");
    display.display();
    delay(5000);
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    digitalWrite(hydroPowPin, LOW);
    cardOff();
    deepSleepFor(3600000L);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  }

  // power cards on
  for (int n=0; n<1; n++){
    digitalWrite(sdPowSelect[n], HIGH);
  }
  cardIsOn = true;
  delay(1000);

  // Initialize the SD card
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.setMISO(12);

  // Start the card and count its free space. A failure right after a reset
  // is often temporary, so retry with a power cycle before giving up.
  int32_t volFree = 0;
  boolean cardBegun = 0;
  for(int attempt = 0; attempt < 5; attempt++){
    watchdogRefresh();
    if(attempt > 0){
      Serial.print("Card retry ");
      Serial.println(attempt);
      sdPowerCycle(0);
    }
    setStep(STEP_CARD_START);
    cardBegun = sd.begin(chipSelect[0]);
    if(cardBegun){
      setStep(STEP_FREE_COUNT);
      volFree = sd.vol()->freeClusterCount();
      if(volFree == 0){  // 0 is also returned if any read fails while counting
        delay(3000);
        volFree = sd.vol()->freeClusterCount();
      }
      if(volFree > 0) break;
    }
  }

  // A free count of 0 (exFAT) or -1 (FAT32) means a read failed. Restart and try again,
  // as for a card that does not start.
  if(cardBegun & (volFree <= 0)){
    Serial.println("Unable to count free space");
    display.print("Card Read Fail");
    display.display();
    delay(5000);
    resetFunc();
  }
  if(!cardBegun){
    Serial.println("Unable to access the SD card");
    display.print("Card Fail");
    display.display();
    delay(5000);
    resetFunc();  // power-cycle the card and restart rather than halting
  }

  sd.card()->readCID(&cid);
  if(sd.fatType() == FAT_TYPE_EXFAT) strcpy(fsName, "exFAT");
  else sprintf(fsName, "FAT%d", sd.fatType());
  clusterKB = sd.vol()->bytesPerCluster() / 1024;
  cardGB = sd.card()->sectorCount() * 512.0 / 1e9;
  setStep(STEP_DISPLAY);
  display.printf("%s %luKB clusters\n", fsName, clusterKB);
  display.printf("Card %.0fGB %c%c%c%c%c\n", cardGB, cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4]);
  display.printf("SN %08lX %d/%d\n", cid.psn, cid.mdt_month, 2000 + (cid.mdt_year_high << 4) + cid.mdt_year_low);
  display.display();

  // File size exactly as the logger computes it
  nbufs_per_file = (long) (ceil(((rec_dur * audio_srate / 256.0) / (float) NREC)) * (float) NREC) * NCHAN;
  writesPerFile = nbufs_per_file / NREC;
  fileBytes = 44 + writesPerFile * NREC * 512;

  if(!resume){
    // Fill the card, keeping the logger's 200 MB margin
    uint32_t bytesPerCluster = sd.vol()->bytesPerCluster();
    uint32_t clustersPerFile = (fileBytes + bytesPerCluster - 1) / bytesPerCluster;
    uint32_t marginClusters = 200000000UL / bytesPerCluster;
    st.filesToWrite = ((uint32_t) volFree > marginClusters) ? ((uint32_t) volFree - marginClusters) / clustersPerFile : 0;
    if(testGB > 0 && st.filesToWrite > testGB * 1e9 / fileBytes) st.filesToWrite = testGB * 1e9 / fileBytes;
    st.targetGB = (float) st.filesToWrite * fileBytes / 1e9;
    if(st.filesToWrite == 0) halt("Card full");

    // An earlier run on this card used the same simulated dates: start a year later so the
    // file names do not clash (open with O_EXCL fails on an existing name)
    st.t = 1793491200;  // 2026-11-01 00:00:00
    setStep(STEP_EXISTS);
    while(sd.exists(folderName(st.t))) st.t += 366UL * 86400;
  }

  makeHeader();
  for(unsigned int i = 0; i < sizeof(buffer); i++) buffer[i] = random(256);  // stand-in for audio
  SdFile::dateTimeCallback(file_date_time);

  Serial.println();
  Serial.printf("SD write test: %s, %lu KB clusters, %.1f GB card, %c%c%c%c%c SN %08lX\n",
    fsName, clusterKB, cardGB, cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4], cid.psn);
  Serial.printf("%lu files of %lu bytes (%.1f GB), %lu writes of %d bytes each, fast fill %.0f GB%s\n",
    st.filesToWrite, fileBytes, st.targetGB, writesPerFile, NREC * 512, fastFillGB, resume ? ", resumed" : "");
  Serial.println("file,GB,mode,open_ms,close_ms,max_write_ms,dropped_ms,write_errors,card_restarts,close_errors");

  delay(3000);
  watchdogRefresh();
  if(!resume){
    startMillis = millis();
    elapsedBase = 0;
    st.magic = STATE_RUNNING;
  }
  segStartMillis = millis();
  segStartBytes = st.bytesWritten;
  saveState();
  writeResults();
  showStats();
}

void loop() {
  if(finished) return;
  watchdogRefresh();
  if(stopPressed()) stopped = true;
  if(stopped || st.filesWritten >= st.filesToWrite){
    finished = true;
    endSegment();
    st.magic = STATE_DONE;  // a restart from here starts a new run
    saveState();
    writeResults();
    showStats();
    watchdogStop();  // stays stopped until the power is cycled
    Serial.println(stopped ? "Stopped: UP+DOWN" : "Done");
    return;
  }

  if(filling()){
    writeFile(false);  // back-to-back, no audio timing
    return;
  }

  // As the logger: start on schedule, record, then sleep until the next start
  if(rec_int > 0){
    if(!scheduled){
      nextStartMs = millis();
      scheduled = true;
    }
    waitForStart();
    if(stopped) return;
    nextStartMs += (rec_dur + rec_int) * 1000;  // startTime = stopTime + rec_int
  }
  writeFile(true);
  if(rec_int > 0 && !stopped){
    delay(100);  // stopRecording()
    sleepUntilStart();
  }
}

boolean filling(){
  return st.bytesWritten < fastFillGB * 1e9;
}

boolean stopPressed(){
  return (digitalRead(UP)==0) & (digitalRead(DOWN)==0);
}

char *folderName(time_t when){
  sprintf(dirname, "/%04d-%02d-%02d", year(when), month(when), day(when));
  return dirname;
}

char *fileName(time_t when){
  sprintf(filename,"%04d%02d%02dT%02d%02d%02d_%lu%lu_%2.1fdB_%2.1fV_ver%s.wav", year(when), month(when), day(when), hour(when), minute(when), second(when), myID[0], myID[1], gainDb, voltage, codeVersion);
  return filename;
}

// One file, as FileInit(), continueRecording() and the file close do it.
// paced: each write waits until that much audio would have arrived.
void writeFile(boolean paced){
  // a folder for each day, as FileInit()
  long today = year(st.t) * 10000L + month(st.t) * 100 + day(st.t);
  if (folderDate != today){
    setStep(STEP_FOLDER);
    folderDate = today;
    folderName(st.t);
    sd.mkdir(dirname);
    sd.chdir(dirname);
  }
  fileName(st.t);

  setStep(STEP_OPEN);
  uint32_t ms = millis();
  if(!file.open(filename, O_WRITE | O_CREAT | O_EXCL)){
    setStep(STEP_EXISTS);
    if(sd.exists(filename)){  // left by an earlier run or a restart, not a card error
      st.t += rec_dur + rec_int;  // next file name
      st.skippedNames++;
      saveState();
      return;
    }
    st.openErrors++;
    Serial.printf("open failed %s\n", filename);
    // as FileInit(): try names from a count; restart only if those fail too
    setStep(STEP_OPEN);
    if(dirname[0]) sd.chdir(dirname);
    boolean opened = false;
    for(int tries = 0; (tries < 20) & !opened; tries++){
      file_count += 1;
      sprintf(filename,"F%06lu.wav",file_count);
      Serial.println(filename);
      opened = file.open(filename, O_WRITE | O_CREAT | O_EXCL);
      if(!opened) delay(10);
    }
    if(!opened){
      st.t += rec_dur + rec_int;
      resetFunc();
    }
  }
  setCardFailCount(0);  // the card works again
  uint32_t openMs = millis() - ms;
  if(openMs > st.maxOpenMs) st.maxOpenMs = openMs;
  setStep(STEP_HEADER);
  file.write((uint8_t *)&wav_hdr, 44);

  // Duty cycle: the queues start after the file is open (startRecording).
  // Continuous: audio kept arriving while the last file closed and this one opened.
  if(paced && (rec_int > 0 || !audioRunning)){
    audioLastUs = micros();
    audioProduced = 0;
    audioConsumed = 0;
    audioRunning = true;
  }
  if(!paced) audioRunning = false;
  double fileDropped = 0;
  if(paced) fileDropped += audioOverflow();

  uint32_t fileMaxUs = 0;
  setStep(STEP_WRITE);
  for(uint32_t w = 0; w < writesPerFile; w++){
    watchdogRefresh();  // the logger refreshes at the top of loop(), at least once per write
    RFSYS_REG5 = w;
    if(paced){
      // wait for NREC blocks of audio, as continueRecording()
      audioTick();
      while(audioProduced < audioConsumed + samplesPerWrite){
        asm("wfi");
        audioTick();
      }
      audioConsumed += samplesPerWrite;  // copied out of the queues before the write
    }
    digitalWrite(LEDGREEN, HIGH);
    uint32_t us = micros();
    size_t n = file.write(buffer, NREC*512);
    us = micros() - us;
    digitalWrite(LEDGREEN, LOW);
    recordWrite(us);
    if(us > fileMaxUs) fileMaxUs = us;
    if(n != NREC*512){
      st.writeErrors++;
      Serial.printf("write failed %s write %lu after %lu ms\n", filename, w, us / 1000);
      st.t += rec_dur + rec_int;
      resetFunc();  // as the logger: restart with the card power cycled, then resume
    }
    st.bytesWritten += NREC*512;
    if(paced) fileDropped += audioOverflow();

    if(stopPressed()){
      // stop recording, as the logger: update the wav header with the length written
      setStep(STEP_CLOSE);
      wav_hdr.dLen = (w + 1) * NREC * 512;
      wav_hdr.rLen = 36 + wav_hdr.dLen;
      file.seek(0);
      file.write((uint8_t *)&wav_hdr, 44);
      if(!file.close()) st.closeErrors++;
      st.bytesWritten += 44;
      stopped = true;
      Serial.printf("stopped in %s after %lu writes\n", filename, w + 1);
      return;
    }
  }

  setStep(STEP_CLOSE);
  ms = millis();
  if(!file.close()) st.closeErrors++;  // the logger does not check this
  uint32_t closeMs = millis() - ms;
  if(closeMs > st.maxCloseMs) st.maxCloseMs = closeMs;

  st.filesWritten++;
  st.bytesWritten += 44;
  st.t += rec_dur + rec_int;

  Serial.printf("%lu,%.3f,%s,%lu,%lu,%lu,%.0f,%lu,%lu,%lu\n", st.filesWritten, st.bytesWritten / 1e9, paced ? "logger" : "fill",
    openMs, closeMs, fileMaxUs / 1000, fileDropped * 1000 / audio_srate, st.writeErrors, st.sdRestarts, st.closeErrors);

  int newSeg = (int) (st.bytesWritten / 1e9 * NSEG / st.targetGB);
  if(newSeg != st.seg && newSeg < NSEG){
    endSegment();
    st.seg = newSeg;
    writeResults();
  }
  saveState();
  if(!paced || rec_int == 0) showStats();  // duty cycle shows it in standby
}

void audioTick(){
  uint32_t now = micros();
  audioProduced += (now - audioLastUs) * (audio_srate / 1e6);
  audioLastUs = now;
}

// Audio beyond what the logger can buffer is lost. Returns the samples dropped.
double audioOverflow(){
  audioTick();
  double backlog = audioProduced - audioConsumed;
  if(backlog <= bufferSamples) return 0;
  double lost = backlog - bufferSamples;
  audioConsumed += lost;
  st.droppedSamples += lost;
  st.dropWrites++;
  return lost;
}

// As the logger after stopRecording(): sleep if the next start is at least 10 s away
// (hydrophone and audio off; the card off too for sleeps of 2 min or more with sdSleep),
// wake wakeahead seconds early, then wait.
void sleepUntilStart(){
  long sleepMs = (long) (nextStartMs - millis()) - wakeahead * 1000L;
  if(sleepMs >= 10000){
    st.sleeps++;
    boolean cardSleep = sdSleep & (sleepMs / 1000 >= SD_SLEEP_SECONDS);  // SD Sleep set to Power off
    if(cardSleep){
      setStep(STEP_CARD_OFF);
      cardOff();
      st.cardSleeps++;
    }
    delay(100);
    deepSleepFor(sleepMs);
    if(cardSleep){
      setStep(STEP_CARD_START);
      if(!cardOn()) resetFunc();  // card will not start: restart and resume
    }
    delay(300);  // give time for Serial to reconnect to USB
  }
}

// Deep sleep in pieces of up to 60 s, as the logger's LPTMR backup wakes. The watchdog is
// paused in deep sleep and refreshed at each wake. Returns early if UP+DOWN is pressed.
void deepSleepFor(long ms){
  setStep(STEP_SLEEP);
  uint32_t wakeMs = millis() + ms;
  while((long) (wakeMs - millis()) > 0){
    watchdogRefresh();
    long chunk = (long) (wakeMs - millis());
    if(chunk > 60000) chunk = 60000;
    lptmr.setTimer(chunk);
    Snooze.deepSleep(config_sleep);   // Snooze adds the sleep time to millis()
    watchdogRefresh();
    if(stopPressed()){
      stopped = true;
      return;
    }
  }
}

// Standby until the scheduled start; the logger updates its display here
void waitForStart(){
  showStats();
  setStep(STEP_WAIT);
  while((long) (nextStartMs - millis()) > 0){
    watchdogRefresh();
    if(stopPressed()){
      stopped = true;
      return;
    }
    asm("wfi");
  }
}

void recordWrite(uint32_t us){
  st.writeCount++;
  for(int i = 0; i < NSLOW; i++) if(us > slowMs[i] * 1000UL) st.slowCount[i]++;
  if(us > st.maxWriteUs){
    st.maxWriteUs = us;
    st.maxWriteGB = st.bytesWritten / 1e9;
  }
  uint16_t msVal = us / 1000;
  if(msVal > st.segMaxMs[st.seg]) st.segMaxMs[st.seg] = msVal;
  if(us > 100000UL) st.segSlow[st.seg]++;
}

void endSegment(){
  float sec = (millis() - segStartMillis) / 1000.0;
  if(sec > 0) st.segMBps[st.seg] = (st.bytesWritten - segStartBytes) / 1e6 / sec;
  segStartMillis = millis();
  segStartBytes = st.bytesWritten;
}

uint32_t elapsedSeconds(){
  return elapsedBase + (millis() - startMillis) / 1000;
}

void saveState(){
  st.elapsedSec = elapsedSeconds();
  EEPROM.put(STATE_ADDR, st);
}

void setStep(uint8_t s){
  RFSYS_REG6 = STEP_MARK | (RFSYS_REG6 & 0xFF00) | s;
}

//*********** Copied from LS1-LS2-v4, with the differences noted ***********

// Restart after an SD card error. The logger counts it in EEPROM byte 25; the test counts it
// in its own state, and keeps the step that failed.
void resetFunc(void){
  uint32_t failed = RFSYS_REG6 & 0x7F;
  st.sdRestarts++;
  saveState();
  RFSYS_REG6 = STEP_MARK | (failed << 8) | STEP_RESTART;
  setCardFailCount(cardFailCount() + 1);
  restartLogger();
}

// Close the file, cycle the card's power and restart the processor. During a deployment
// the restart resumes recording (see setup()). The logger closes the file if it is
// recording (mode == 1); the test if one is open.
void restartLogger(){
  watchdogRefresh();
  if(file.isOpen()) file.close();  // close file if open
  cardOff();
  //cycle power on SD cards (in case there are cards in other slots)
  for(int n = 0; n<1; n++){
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

// Switch the card off and on again
void sdPowerCycle(int n){
  cardRelease(n);
  delay(2000);
  digitalWrite(sdPowSelect[n], HIGH);
  cardIsOn = true;
  delay(1000);  // sd.begin() reconnects the SPI pins
}

// Take the SPI pins off the card, hold CS low and cut its power. Pins left on SPI (or only
// written low while still on SPI) keep powering the card through its protection diodes.
void cardRelease(int n){
  pinMode(7, INPUT_DISABLE);   // MOSI
  pinMode(12, INPUT_DISABLE);  // MISO
  pinMode(14, INPUT_DISABLE);  // SCK
  pinMode(chipSelect[n], OUTPUT);
  digitalWrite(chipSelect[n], LOW);
  digitalWrite(sdPowSelect[n], LOW);
  cardIsOn = false;
}

// Let the card finish any operation in progress, then switch it off
void cardOff(){
  if(cardIsOn && sd.card()){
    uint32_t busyStart = millis();
    while(sd.card()->isBusy() && (millis() - busyStart < 2000)) delay(10);
  }
  cardRelease(0);
}

// Power the card and start it, with up to 4 power-cycle retries, then go back to the
// current folder. Returns false if it will not start.
boolean cardOn(){
  digitalWrite(sdPowSelect[0], HIGH);
  cardIsOn = true;
  delay(1000);
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.setMISO(12);
  for(int attempt = 0; attempt < 5; attempt++){
    watchdogRefresh();
    if(attempt > 0){
      Serial.print("Card retry ");
      Serial.println(attempt);
      sdPowerCycle(0);
    }
    if(sd.begin(chipSelect[0])){
      if(dirname[0]) sd.chdir(dirname);  // back to the day folder
      return true;
    }
  }
  return false;
}

// Card failures in a row, kept in the system register file: it survives restarts but not a
// power-up, and needs no EEPROM writes. A marker in the top half shows it has been set.
#define CARD_FAIL_MARK 0x4C530000
int cardFailCount(){
#ifdef SIM_SCGC5_REGFILE
  SIM_SCGC5 |= SIM_SCGC5_REGFILE;
#endif
  uint32_t v = RFSYS_REG7;
  return ((v & 0xFFFF0000) == CARD_FAIL_MARK) ? (v & 0xFFFF) : 0;
}

void setCardFailCount(int n){
#ifdef SIM_SCGC5_REGFILE
  SIM_SCGC5 |= SIM_SCGC5_REGFILE;
#endif
  if(n > 0xFFFF) n = 0xFFFF;
  RFSYS_REG7 = CARD_FAIL_MARK | n;
}

// Hardware watchdog: restarts the processor if loop(), the menu or a card retry stops for 60 s.
// It runs on the 1 kHz LPO and is stopped in deep sleep (STOPEN clear; LLS is a stop mode), so a
// sleep of any length cannot trigger it, but a sleep that keeps waking at once soon does.
// The Teensy core leaves it disabled with ALLOWUPDATE set, so it can be set up here.
// The test adds IRQRSTEN: an interrupt just before the reset saves where the program was.
void watchdogStart(){
  noInterrupts();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  __asm__ volatile ("nop");
  __asm__ volatile ("nop");
  WDOG_TOVALH = 0;
  WDOG_TOVALL = 60000;  // 60 s at 1 kHz
  WDOG_PRESC = 0;       // divide by 1 (the reset value divides by 5)
  WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_IRQRSTEN;  // LPO clock
  interrupts();
  NVIC_ENABLE_IRQ(IRQ_WDOG);
}

void watchdogRefresh(){
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts();
}

// For the stops: a stopped logger stays stopped until the power is cycled
void watchdogStop(){
  noInterrupts();
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  __asm__ volatile ("nop");
  __asm__ volatile ("nop");
  WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;
  interrupts();
}

//*************************************************************************

// Watchdog interrupt, 256 bus clocks before the reset: save the program counter and return
// address from the interrupted code's stack frame. It does not run if the program is stuck
// with interrupts off or in a fault handler.
extern "C" void __attribute__((used)) watchdogSaveFrame(uint32_t *frame){
  RFSYS_REG4 = frame[6];  // PC
  RFSYS_REG3 = frame[5];  // LR
  RFSYS_REG6 |= PC_SAVED;
  while(1);  // the reset follows
}

extern "C" void __attribute__((naked)) watchdog_isr(void){
  __asm__ volatile(
    "mrs r0, msp\n"
    "b watchdogSaveFrame\n");
}

void makeHeader(){
  //intialize stereo .wav file header, as FileInit()
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
}

void file_date_time(uint16_t* date, uint16_t* time)
{
  *date=FAT_DATE(year(st.t),month(st.t),day(st.t));
  *time=FAT_TIME(hour(st.t),minute(st.t),second(st.t));
}

void printDuration(Print &p, uint32_t sec){
  p.printf("%luh%02lum", sec / 3600, (sec / 60) % 60);
}

// n,cause,step,write_in_file,file,GB,PC,LR
void printRestart(Print &p, uint32_t n, RestartRec &r){
  p.printf("%lu,%s,%s,%lu,%lu,%.2f,", n, causeName[r.cause], stepName[r.step], r.write, r.file, r.gb);
  if(r.pcSaved) p.printf("%08lX,%08lX", r.pc, r.lr);
  else p.print("-,-");
}

void showStats(){
  setStep(STEP_DISPLAY);
  float gb = st.bytesWritten / 1e9;
  uint32_t elapsed = elapsedSeconds();
  float rate = elapsed ? st.bytesWritten / 1e6 / elapsed : 0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%s %luKB ", fsName, clusterKB);
  if(stopped) display.println("STOPPED");
  else if(finished) display.println("DONE");
  else if(filling()) display.println("fill");
  else display.println("logger");
  display.printf("%.1f/%.1fGB %d%%\n", gb, st.targetGB, (int) (100 * gb / st.targetGB));
  display.printf("%.2fMB/s max%lums\n", rate, st.maxWriteUs / 1000);
  display.printf(">50:%lu >100:%lu\n", st.slowCount[0], st.slowCount[1]);
  display.printf(">250:%lu >600:%lu >1s:%lu\n", st.slowCount[2], st.slowCount[3], st.slowCount[4]);
  display.printf("W%lu O%lu C%lu R%lu WD%lu", st.writeErrors, st.openErrors, st.closeErrors, st.sdRestarts, st.wdogResets);
  if(st.wdogResets) display.printf(" %s", stepName[st.lastWdogStep]);
  display.println();
  display.printf("Dr%.1fs Op%lu Cl%lu\n", st.droppedSamples / audio_srate, st.maxOpenMs, st.maxCloseMs);
  if(finished){
    display.print("Safe to turn off");
  }
  else{
    printDuration(display, elapsed);
    if(rate > 0){
      display.print(" ETA ");
      printDuration(display, (uint32_t) ((st.targetGB - gb) * 1000 / rate));
    }
  }
  display.display();
}

// Summary in the card's root folder, rewritten at each twentieth of the test and at the end
void writeResults(){
  setStep(STEP_RESULTS);
  FsFile rf;
  if(!rf.open("/WRTEST.TXT", O_WRITE | O_CREAT | O_TRUNC)) return;
  uint32_t elapsed = elapsedSeconds();
  rf.printf("SD write test (mirrors LS1-LS2-v4 %s)\r\n", codeVersion);
  rf.printf("State: %s\r\n", stopped ? "stopped with UP+DOWN" : finished ? "done" : "running");
  rf.printf("Card: %c%c%c%c%c SN %08lX made %d/%d, %.1f GB, %s, %lu KB clusters\r\n",
    cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4], cid.psn,
    cid.mdt_month, 2000 + (cid.mdt_year_high << 4) + cid.mdt_year_low, cardGB, fsName, clusterKB);
  rf.printf("Settings: rec %lu s, sleep %lu s, %.0f Hz, %d ch, card off in sleeps of %d s or more: %s, fast fill %.0f GB first\r\n",
    rec_dur, rec_int, audio_srate, NCHAN, SD_SLEEP_SECONDS, sdSleep ? "yes" : "no", fastFillGB);
  rf.printf("SdFat timeouts: write %u ms, command %u ms\r\n", SD_WRITE_TIMEOUT, SD_CMD_TIMEOUT);
  rf.printf("Files: %lu of %lu, %lu bytes each, %lu writes of %d bytes per file\r\n", st.filesWritten, st.filesToWrite, fileBytes, writesPerFile, NREC*512);
  rf.printf("Written: %.2f of %.2f GB in ", st.bytesWritten / 1e9, st.targetGB);
  printDuration(rf, elapsed);
  rf.printf(", %.2f MB/s, %lu sleeps, %lu with the card off\r\n", elapsed ? st.bytesWritten / 1e6 / elapsed : 0, st.sleeps, st.cardSleeps);
  rf.printf("Writes: %lu, max %lu ms at %.2f GB\r\n", st.writeCount, st.maxWriteUs / 1000, st.maxWriteGB);
  for(int i = 0; i < NSLOW; i++) rf.printf("  over %u ms: %lu\r\n", slowMs[i], st.slowCount[i]);
  rf.printf("Max open %lu ms, max close %lu ms\r\n", st.maxOpenMs, st.maxCloseMs);
  rf.printf("Audio dropped (estimate, logger timing only): %.1f s over %lu writes\r\n", st.droppedSamples / audio_srate, st.dropWrites);
  rf.printf("Errors: write %lu, open %lu, close %lu\r\n", st.writeErrors, st.openErrors, st.closeErrors);
  rf.printf("Restarts: %lu after card errors, %lu by the watchdog, %lu other\r\n", st.sdRestarts, st.wdogResets, st.otherResets);
  if(st.nRestarts){
    uint32_t first = (st.nRestarts > NLOG) ? st.nRestarts - NLOG : 0;
    rf.printf("Last restarts: n,cause,step,write_in_file,file,GB,PC,LR\r\n");
    for(uint32_t n = first; n < st.nRestarts; n++){
      printRestart(rf, n + 1, st.restartLog[n % NLOG]);
      rf.printf("\r\n");
    }
    rf.printf("PC and LR (watchdog only): arm-none-eabi-addr2line -f -C -e SDWriteTest.ino.elf PC LR, with the .elf of this build\r\n");
  }
  rf.printf("File names skipped (left by an earlier run or a restart): %lu\r\n", st.skippedNames);
  rf.printf("\r\nBy twentieth of the test:\r\npart,from_GB,MB_per_s,max_write_ms,writes_over_100ms\r\n");
  for(int i = 0; i < NSEG; i++){
    rf.printf("%d,%.1f,%.2f,%u,%lu\r\n", i + 1, i * st.targetGB / NSEG, st.segMBps[i], st.segMaxMs[i], st.segSlow[i]);
  }
  rf.close();
}

void halt(const char *msg){
  watchdogStop();
  Serial.println(msg);
  display.println(msg);
  display.display();
  while(1);
}
