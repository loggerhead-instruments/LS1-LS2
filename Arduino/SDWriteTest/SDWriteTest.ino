//
// SD card write test for LS1/LS2 v4 boards
//
// Writes files to the card the way LS1-LS2-v4 records a duty cycle:
// - the same SdFat calls (sd.begin, mkdir/chdir, open, write, close), a 44-byte header
//   followed by 16384-byte writes (NREC * 512), the same file size, names and monthly folders
// - each write happens when that much audio would have arrived (every 85 ms at 48 kHz stereo)
// - between files, the same steps as the logger: 100 ms after closing (stopRecording), deep
//   sleep with the card left powered, wake 4 s early (wakeahead), 300 ms after waking, then
//   standby until the scheduled start. Sleeps under 10 s are skipped, as on the logger.
// - continuous mode (rec_int = 0): close and open the next file while audio keeps arriving
// Audio itself is not recorded; the buffer is a stand-in. The display stays on with the
// results and is only updated between files, when the logger also uses it.
//
// Every write is timed. A line per file goes to USB serial, and a summary is kept in
// WRTEST.TXT in the card's root folder.
//
// Filling a 1 TB card this way takes as long as a deployment. To reach the nearly-full state
// sooner, set fastFillGB: that much is written back-to-back first, then the test runs as the
// logger.
//
// Prepare the card the way you want to test it (Windows quick format, SD Card Formatter
// overwrite, or the SdFat SdFormatter example), then power up the board.
// Compile as for the logger: Teensy 3.2, 96 MHz, with the same SdFat-beta library, so the
// write timeout matches.
//

#include <SPI.h>
#include <Wire.h>
#include "SdFat.h"
#include <Snooze.h>
#include <TimeLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

//*********** Settings: match the deployment being tested ***********
const unsigned long rec_dur = 300;     // record duration (s)
const unsigned long rec_int = 10;      // sleep between recordings (s); 0 = continuous, all files in root
const float audio_srate = 48000;       // sample rate (Hz)
const int NCHAN = 2;                   // channels
const float fastFillGB = 0;            // write this many GB back-to-back first, then run as the logger
const float testGB = 0;                // stop after this many GB in total; 0 = fill the card
//*******************************************************************

#define NREC 32       // same as LS1-LS2-v4: each write is NREC * 512 = 16384 bytes
int wakeahead = 4;    // same as LS1-LS2-v4
char codeVersion[5] = "4.40";  // only used in file names, as the logger does
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
int folderMonth = -1;
uint32_t myID[2];
unsigned long nbufs_per_file;
uint32_t writesPerFile;
uint32_t fileBytes;
const float samplesPerWrite = NREC * 512 / (2 * NCHAN);  // per channel

// Simulated clock for file names, folders and timestamps: advances by rec_dur + rec_int per file
time_t t = 1793491200;  // 2026-11-01 00:00:00

// Card
char fsName[8];
uint32_t clusterKB;
float cardGB;
cid_t cid;
uint32_t filesToWrite;
float targetGB;

// Schedule, in millis(); Snooze adds the LPTMR sleep time to millis()
uint32_t nextStartMs = 0;
boolean scheduled = false;

// Audio arriving while recording
boolean audioRunning = false;
uint32_t audioLastUs;
double audioProduced, audioConsumed;  // samples per channel
double droppedSamples = 0;
uint32_t dropWrites = 0;

// Statistics
const uint16_t slowMs[] = {50, 100, 250, 600, 1000};  // 600 ms is SdFat's default write timeout
#define NSLOW 5
uint32_t slowCount[NSLOW];
uint32_t writeCount = 0;
uint32_t maxWriteUs = 0;
float maxWriteGB = 0;
uint32_t maxOpenMs = 0, maxCloseMs = 0;
uint32_t writeErrors = 0, openErrors = 0, restarts = 0;
uint32_t filesWritten = 0;
uint32_t sleeps = 0;
uint64_t bytesWritten = 0;
uint32_t startMillis;
boolean cardFailed = false;
boolean finished = false;

// Results by twentieths of the test, to see whether writes slow down as the card fills
#define NSEG 20
uint16_t segMaxMs[NSEG];
uint32_t segSlow[NSEG];   // writes over 100 ms
float segMBps[NSEG];
int seg = 0;
uint32_t segStartMillis;
uint64_t segStartBytes = 0;

void setup() {
  myID[0] = SIM_UIDMH;
  myID[1] = SIM_UIDML;

  Serial.begin(115200);
  delay(500);

  pinMode(LEDGREEN, OUTPUT);
  pinMode(hydroPowPin, OUTPUT);
  digitalWrite(hydroPowPin, LOW);  // hydrophone not needed
  pinMode(SGTL_EN, OUTPUT);
  digitalWrite(SGTL_EN, HIGH);     // as when recording; keeps I2C working
  pinMode(SDPOW1, OUTPUT);
  digitalWrite(SDPOW1, LOW);

  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  delay(140);
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SD Write Test");
  display.printf("%lus/%lus %.0fkHz %dch\n", rec_dur, rec_int, audio_srate / 1000, NCHAN);
  display.printf("Timeout wr%u cmd%ums\n", SD_WRITE_TIMEOUT, SD_CMD_TIMEOUT);
  display.display();

  // Start the card as manualSettings() does
  digitalWrite(SDPOW1, HIGH);
  delay(1000);
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.setMISO(12);
  boolean ok = sd.begin(CS1);
  for(int attempt = 1; attempt < 5 && !ok; attempt++){
    sdPowerCycle();
    ok = sd.begin(CS1);
  }
  if(!ok) halt("Card Fail");

  sd.card()->readCID(&cid);
  if(sd.fatType() == FAT_TYPE_EXFAT) strcpy(fsName, "exFAT");
  else sprintf(fsName, "FAT%d", sd.fatType());
  clusterKB = sd.vol()->bytesPerCluster() / 1024;
  cardGB = sd.card()->sectorCount() * 512.0 / 1e9;
  display.printf("%s %luKB clusters\n", fsName, clusterKB);
  display.printf("Card %.0fGB %c%c%c%c%c\n", cardGB, cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4]);
  display.printf("SN %08lX %d/%d\n", cid.psn, cid.mdt_month, 2000 + (cid.mdt_year_high << 4) + cid.mdt_year_low);
  display.println("Counting free space");
  display.display();

  int32_t volFree = sd.vol()->freeClusterCount();
  if(volFree <= 0){
    delay(3000);
    volFree = sd.vol()->freeClusterCount();
  }
  if(volFree <= 0) halt("Free count fail");

  // File size exactly as the logger computes it
  nbufs_per_file = (long) (ceil(((rec_dur * audio_srate / 256.0) / (float) NREC)) * (float) NREC) * NCHAN;
  writesPerFile = nbufs_per_file / NREC;
  fileBytes = 44 + writesPerFile * NREC * 512;

  // Fill the card, keeping the logger's 200 MB margin
  uint32_t bytesPerCluster = sd.vol()->bytesPerCluster();
  uint32_t clustersPerFile = (fileBytes + bytesPerCluster - 1) / bytesPerCluster;
  uint32_t marginClusters = 200000000UL / bytesPerCluster;
  filesToWrite = ((uint32_t) volFree > marginClusters) ? ((uint32_t) volFree - marginClusters) / clustersPerFile : 0;
  if(testGB > 0 && filesToWrite > testGB * 1e9 / fileBytes) filesToWrite = testGB * 1e9 / fileBytes;
  targetGB = (float) filesToWrite * fileBytes / 1e9;
  if(filesToWrite == 0) halt("Card full");

  makeHeader();
  for(unsigned int i = 0; i < sizeof(buffer); i++) buffer[i] = random(256);  // stand-in for audio
  SdFile::dateTimeCallback(file_date_time);

  Serial.println();
  Serial.printf("SD write test: %s, %lu KB clusters, %.1f GB card, %c%c%c%c%c SN %08lX\n",
    fsName, clusterKB, cardGB, cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4], cid.psn);
  Serial.printf("%lu files of %lu bytes (%.1f GB), %lu writes of %d bytes each, fast fill %.0f GB\n",
    filesToWrite, fileBytes, targetGB, writesPerFile, NREC * 512, fastFillGB);
  Serial.println("file,GB,mode,open_ms,close_ms,max_write_ms,dropped_ms,write_errors,restarts");

  delay(3000);
  startMillis = millis();
  segStartMillis = startMillis;
  writeResults();
  showStats();
}

void loop() {
  if(finished) return;
  if(cardFailed || filesWritten >= filesToWrite){
    finished = true;
    endSegment();
    writeResults();
    showStats();
    Serial.println(cardFailed ? "Stopped: card failed" : "Done");
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
    nextStartMs += (rec_dur + rec_int) * 1000;  // startTime = stopTime + rec_int
  }
  writeFile(true);
  if(rec_int > 0){
    delay(100);  // stopRecording()
    sleepUntilStart();
  }
}

boolean filling(){
  return bytesWritten < fastFillGB * 1e9;
}

// One file, as FileInit(), continueRecording() and the file close do it.
// paced: each write waits until that much audio would have arrived.
void writeFile(boolean paced){
  // use folders for duty cycle record, as FileInit()
  if (folderMonth != month(t) & (rec_int > 0)){
    folderMonth = month(t);
    sprintf(dirname, "/%04d-%02d", year(t), folderMonth);
    sd.mkdir(dirname);
    sd.chdir(dirname);
  }
  sprintf(filename,"%04d%02d%02dT%02d%02d%02d_%lu%lu_%2.1fdB_%2.1fV_ver%s.wav", year(t), month(t), day(t), hour(t), minute(t), second(t), myID[0], myID[1], gainDb, voltage, codeVersion);

  uint32_t ms = millis();
  if(!file.open(filename, O_WRITE | O_CREAT | O_EXCL)){
    openErrors++;
    Serial.printf("open failed %s\n", filename);
    t += rec_dur + rec_int;  // next file name
    recoverCard();
    return;
  }
  uint32_t openMs = millis() - ms;
  if(openMs > maxOpenMs) maxOpenMs = openMs;
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
  for(uint32_t w = 0; w < writesPerFile; w++){
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
      writeErrors++;
      Serial.printf("write failed %s write %lu after %lu ms\n", filename, w, us / 1000);
      file.close();
      t += rec_dur + rec_int;
      audioRunning = false;
      recoverCard();
      return;
    }
    bytesWritten += NREC*512;
    if(paced) fileDropped += audioOverflow();
  }

  ms = millis();
  file.close();
  uint32_t closeMs = millis() - ms;
  if(closeMs > maxCloseMs) maxCloseMs = closeMs;

  filesWritten++;
  bytesWritten += 44;
  t += rec_dur + rec_int;

  Serial.printf("%lu,%.3f,%s,%lu,%lu,%lu,%.0f,%lu,%lu\n", filesWritten, bytesWritten / 1e9, paced ? "logger" : "fill",
    openMs, closeMs, fileMaxUs / 1000, fileDropped * 1000 / audio_srate, writeErrors, restarts);

  int newSeg = (int) (bytesWritten / 1e9 * NSEG / targetGB);
  if(newSeg != seg && newSeg < NSEG){
    endSegment();
    seg = newSeg;
    writeResults();
  }
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
  droppedSamples += lost;
  dropWrites++;
  return lost;
}

// As the logger after stopRecording(): sleep if the next start is at least 10 s away
// (hydrophone and audio off, card left powered), wake wakeahead seconds early, then wait.
void sleepUntilStart(){
  long sleepMs = (long) (nextStartMs - millis()) - wakeahead * 1000L;
  if(sleepMs >= 10000){
    sleeps++;
    delay(100);
    uint32_t wakeMs = millis() + sleepMs;
    while((long) (wakeMs - millis()) > 0){
      long chunk = (long) (wakeMs - millis());
      if(chunk > 60000) chunk = 60000;  // the logger's LPTMR wakes at least once a minute
      lptmr.setTimer(chunk);
      Snooze.deepSleep(config_sleep);   // Snooze adds the sleep time to millis()
    }
    delay(300);  // give time for Serial to reconnect to USB
  }
}

// Standby until the scheduled start; the logger updates its display here
void waitForStart(){
  showStats();
  while((long) (nextStartMs - millis()) > 0) asm("wfi");
}

void recordWrite(uint32_t us){
  writeCount++;
  for(int i = 0; i < NSLOW; i++) if(us > slowMs[i] * 1000UL) slowCount[i]++;
  if(us > maxWriteUs){
    maxWriteUs = us;
    maxWriteGB = bytesWritten / 1e9;
  }
  uint16_t msVal = us / 1000;
  if(msVal > segMaxMs[seg]) segMaxMs[seg] = msVal;
  if(us > 100000UL) segSlow[seg]++;
}

void endSegment(){
  float sec = (millis() - segStartMillis) / 1000.0;
  if(sec > 0) segMBps[seg] = (bytesWritten - segStartBytes) / 1e6 / sec;
  segStartMillis = millis();
  segStartBytes = bytesWritten;
}

// After an error, do what the logger does: let the card finish, power-cycle it and
// start it again (resetFunc() and the retries in manualSettings())
void recoverCard(){
  restarts++;
  if(sd.card()){
    uint32_t busyStart = millis();
    while(sd.card()->isBusy() && (millis() - busyStart < 2000)) delay(10);
  }
  for(int attempt = 0; attempt < 5; attempt++){
    sdPowerCycle();
    if(sd.begin(CS1)){
      if(rec_int > 0) sd.chdir(dirname);
      return;
    }
  }
  cardFailed = true;
}

// Switch the card off and on again, as sdPowerCycle() in LS1-LS2-v4
void sdPowerCycle(){
  pinMode(7, INPUT_DISABLE);   // MOSI
  pinMode(12, INPUT_DISABLE);  // MISO
  pinMode(14, INPUT_DISABLE);  // SCK
  pinMode(CS1, OUTPUT);
  digitalWrite(CS1, LOW);
  digitalWrite(SDPOW1, LOW);
  delay(2000);
  digitalWrite(SDPOW1, HIGH);
  delay(1000);  // sd.begin() reconnects the SPI pins
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
  *date=FAT_DATE(year(t),month(t),day(t));
  *time=FAT_TIME(hour(t),minute(t),second(t));
}

void printDuration(Print &p, uint32_t sec){
  p.printf("%luh%02lum", sec / 3600, (sec / 60) % 60);
}

void showStats(){
  float gb = bytesWritten / 1e9;
  uint32_t elapsed = (millis() - startMillis) / 1000;
  float rate = elapsed ? bytesWritten / 1e6 / elapsed : 0;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("%s %luKB ", fsName, clusterKB);
  if(cardFailed) display.println("CARD FAIL");
  else if(finished) display.println("DONE");
  else if(filling()) display.println("fill");
  else display.println("logger");
  display.printf("%.1f/%.1fGB %d%%\n", gb, targetGB, (int) (100 * gb / targetGB));
  display.printf("%.2fMB/s max%lums\n", rate, maxWriteUs / 1000);
  display.printf(">50:%lu >100:%lu\n", slowCount[0], slowCount[1]);
  display.printf(">250:%lu >600:%lu >1s:%lu\n", slowCount[2], slowCount[3], slowCount[4]);
  display.printf("Err W%lu O%lu Rst%lu\n", writeErrors, openErrors, restarts);
  display.printf("Drop%.1fs Op%lu Cl%lu\n", droppedSamples / audio_srate, maxOpenMs, maxCloseMs);
  printDuration(display, elapsed);
  if(!finished && rate > 0){
    display.print(" ETA ");
    printDuration(display, (uint32_t) ((targetGB - gb) * 1000 / rate));
  }
  display.display();
}

// Summary in the card's root folder, rewritten at each twentieth of the test and at the end
void writeResults(){
  FsFile rf;
  if(!rf.open("/WRTEST.TXT", O_WRITE | O_CREAT | O_TRUNC)) return;
  uint32_t elapsed = (millis() - startMillis) / 1000;
  rf.printf("SD write test (mirrors LS1-LS2-v4 %s)\r\n", codeVersion);
  rf.printf("State: %s\r\n", cardFailed ? "card failed" : finished ? "done" : "running");
  rf.printf("Card: %c%c%c%c%c SN %08lX made %d/%d, %.1f GB, %s, %lu KB clusters\r\n",
    cid.pnm[0], cid.pnm[1], cid.pnm[2], cid.pnm[3], cid.pnm[4], cid.psn,
    cid.mdt_month, 2000 + (cid.mdt_year_high << 4) + cid.mdt_year_low, cardGB, fsName, clusterKB);
  rf.printf("Settings: rec %lu s, sleep %lu s, %.0f Hz, %d ch, fast fill %.0f GB first\r\n", rec_dur, rec_int, audio_srate, NCHAN, fastFillGB);
  rf.printf("SdFat timeouts: write %u ms, command %u ms\r\n", SD_WRITE_TIMEOUT, SD_CMD_TIMEOUT);
  rf.printf("Files: %lu of %lu, %lu bytes each, %lu writes of %d bytes per file\r\n", filesWritten, filesToWrite, fileBytes, writesPerFile, NREC*512);
  rf.printf("Written: %.2f of %.2f GB in ", bytesWritten / 1e9, targetGB);
  printDuration(rf, elapsed);
  rf.printf(", %.2f MB/s, %lu sleeps\r\n", elapsed ? bytesWritten / 1e6 / elapsed : 0, sleeps);
  rf.printf("Writes: %lu, max %lu ms at %.2f GB\r\n", writeCount, maxWriteUs / 1000, maxWriteGB);
  for(int i = 0; i < NSLOW; i++) rf.printf("  over %u ms: %lu\r\n", slowMs[i], slowCount[i]);
  rf.printf("Max open %lu ms, max close %lu ms\r\n", maxOpenMs, maxCloseMs);
  rf.printf("Audio dropped (estimate, logger timing only): %.1f s over %lu writes\r\n", droppedSamples / audio_srate, dropWrites);
  rf.printf("Errors: write %lu, open %lu, card restarts %lu\r\n", writeErrors, openErrors, restarts);
  rf.printf("\r\nBy twentieth of the test:\r\npart,from_GB,MB_per_s,max_write_ms,writes_over_100ms\r\n");
  for(int i = 0; i < NSEG; i++){
    rf.printf("%d,%.1f,%.2f,%u,%lu\r\n", i + 1, i * targetGB / NSEG, segMBps[i], segMaxMs[i], segSlow[i]);
  }
  rf.close();
}

void halt(const char *msg){
  Serial.println(msg);
  display.println(msg);
  display.display();
  while(1);
}
