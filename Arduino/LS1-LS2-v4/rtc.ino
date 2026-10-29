int RTCAddress = 0x68;

volatile byte ds_second;
volatile byte ds_minute;
volatile byte ds_hour;
volatile byte ds_day;
volatile byte ds_month;
volatile byte ds_year;

#define SECONDS_IN_MINUTE 60
#define SECONDS_IN_HOUR 3600
#define SECONDS_IN_DAY 86400
#define SECONDS_IN_YEAR 31536000
#define SECONDS_IN_LEAP 31622400

void setTime2(int thour,int tminute,int tsecond,int tday,int tmonth,int tyear)
{
  Wire.beginTransmission(RTCAddress);
  Wire.write(0x00);  //starting address is 00
  Wire.write(((tsecond/10)<<4) | ((tsecond%10)&0x0F));  //seconds, 
  Wire.write(((tminute/10)<<4) | ((tminute%10)&0x0F));  //min
  Wire.write((((thour/10)&0x03)<<4) | ((thour)%10&0x0F)); //hour  0x20 will set to 12 hour AM/PM mode
  Wire.write(0x01); // day of week(1-7)
  Wire.write(((tday/10)<<4) | ((tday%10)&0x0F)); // date (1-31)
  Wire.write((tmonth/10)<<4 | ((tmonth%10)&0x0F)); //  month
  Wire.write((tyear/10)<<4 | ((tyear%10)&0x0F)); //  year 2010=0x0A
  int ecode=Wire.endTransmission(); //end transmission
//  Serial.println(ecode);

  // keep the backup clocks on the time that was just set
  tmElements_t tm;
  tm.Second = tsecond;
  tm.Minute = tminute;
  tm.Hour = thour;
  tm.Day = tday;
  tm.Month = tmonth;
  tm.Year = tyear + 30U;
  time_t newTime = makeTime(tm);
  if(internalRtcOK) Teensy3Clock.set(newTime);
  setSoftTime(newTime);
}

void set_alarm(int thour, int tminute)
{
  Wire.beginTransmission(RTCAddress);
  Wire.write(0x07); //0x0B Alarm1
  Wire.write(0x00); //seconds
  Wire.write(((tminute/10)<<4) | ((tminute%10)&0x0F)); // Minutes
  Wire.write((((thour/10)&0x03)<<4) | ((thour)%10&0x0F)); //hour
  Wire.write(0x80); //alarm when hour minutes and seconds match
  Wire.endTransmission();

  Wire.beginTransmission(RTCAddress);
  Wire.write(0x0E); //Control register
  Wire.write(0x01); //enable Alarm 1 interrupt
  Wire.write(0x00); //clear existing interrupts
  Wire.endTransmission();
}

void set_alarm_at(time_t wakeTime)
{
  int tsecond = second(wakeTime);
  int tminute = minute(wakeTime);
  int thour = hour(wakeTime);

  reset_alarm(); // clear any pending alarm

  Wire.beginTransmission(RTCAddress);
  Wire.write(0x07); // Alarm 1 registers
  Wire.write(((tsecond/10)<<4) | (tsecond%10));       // seconds, A1M1=0
  Wire.write(((tminute/10)<<4) | (tminute%10));       // minutes, A1M2=0
  Wire.write((((thour/10)&0x03)<<4) | (thour%10));    // hours,   A1M3=0
  Wire.write(0x80);                                     // day,     A1M4=1 (match h:m:s)
  Wire.endTransmission();

  Wire.beginTransmission(RTCAddress);
  Wire.write(0x0E); // Control register
  Wire.write(0x05); // INTCN=1, A1IE=1 (enable Alarm 1 interrupt)
  Wire.write(0x00); // clear status flags
  Wire.endTransmission();
}

void reset_alarm()
{
  Wire.beginTransmission(RTCAddress);
  Wire.write(0x0F); //Control register
  Wire.write(0x00); //clear existing interrupts
  Wire.endTransmission();
}

void disable_alarm()
{
  Wire.beginTransmission(RTCAddress);
  Wire.write(0x0E); // Control register
  Wire.write(0x04); // INTCN=1, alarm interrupts off
  Wire.write(0x00); // clear status flags
  Wire.endTransmission();
}
boolean readRTC()
{
  // Serial.println("Read DS3231: ");
  int i = 0;
  byte buff[7];
  Wire.beginTransmission(RTCAddress); 
  Wire.write(0x00);        //sends address to read from starting with seconds
  Wire.endTransmission(0); //end transmission
  
  Wire.requestFrom(RTCAddress, 7, 0);    // request 7 bytes from clock (seconds through Year)
  int bytesavail=Wire.available();

  while(Wire.available())   // ((Wire.available())&&(i<6))
  { 
    buff[i] = Wire.read();  // receive one byte
    i++;
  }
   Wire.endTransmission();

  if (i==7)  // All bytes received?
  {
      ds_second=(10*(buff[0]>>4)) + ((buff[0]&0x0F)) ;//lower 4 bits is seconds, upper 3 bits are 10 seconds
      ds_minute=(10*(buff[1]>>4)) + ((buff[1]&0x0F)) ;//lower 4 bits is minutes, upper 3 bits are 10 minutes
      ds_hour=(10*((buff[2]>>4)&0x03))+((buff[2]&0x0F));  //bits 4 and 5 are 10 hours; bit 0-3 are hour
      ds_day=(10*(buff[4]>>4))+((buff[4]&0x0F)); 
      ds_month=(10*(buff[5]>>4))+((buff[5]&0x0F)); 
      ds_year=(10*(buff[6]>>4))+((buff[6]&0x0F));
      // reject values a failing chip or an idle bus (all 0xFF) can return
      if(ds_second>59 || ds_minute>59 || ds_hour>23 || ds_day<1 || ds_day>31 || ds_month<1 || ds_month>12) return 0;
     // t =RTCToUNIXTime(ds_year, ds_month, ds_day, ds_hour, ds_minute, ds_second);  
      tmElements_t tm;
      tm.Second = ds_second;
      tm.Minute = ds_minute;
      tm.Hour = ds_hour;
      tm.Day = ds_day;
      tm.Month = ds_month;
      tm.Year = ds_year + 30U;
      t = makeTime(tm);
      // Serial.println(t);
      return 1;
  }
  else return 0;
}

int rtcStatus(){
  byte val[2];
  
  Wire.beginTransmission(RTCAddress); 
  Wire.write(0x0F);        //sends address to read from starting with seconds
  Wire.endTransmission(); //end transmission

  Wire.requestFrom(RTCAddress, 1);
  val[0] = Wire.read();
  //val[1] = Wire.read();
  Wire.endTransmission();

  return val[0];
}

// True if the DS3231 answers with a valid time and is counting
boolean ds3231Running(){
  if(!readRTC()) return false;
  time_t t1 = t;
  uint32_t startMillis = millis();
  while(millis() - startMillis < 2500){
    delay(50);
    if(!readRTC()) return false;
    if(t != t1) return true;
  }
  return false;
}

// Start the internal 32.768 kHz oscillator (26 pF load for the 12.5 pF crystal) if it is
// not already running that way, and check that the internal RTC is counting.
// A CPU restart leaves it running, so the time is kept across resetFunc().
boolean internalRTCRunning(){
  uint32_t rtcCR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P | RTC_CR_OSCE;
  if(RTC_CR != rtcCR){
    RTC_CR = 0;
    delay(100);
    RTC_CR = RTC_CR_SC16P | RTC_CR_SC8P | RTC_CR_SC2P;
    delay(100);
    RTC_CR = rtcCR;
    delay(100);
  }
  if(!(RTC_SR & RTC_SR_TCE) || (RTC_SR & RTC_SR_TIF)) Teensy3Clock.set(DEFAULT_TIME); // start the counter
  uint32_t start = RTC_TSR;
  uint32_t startMillis = millis();
  while(millis() - startMillis < 3000){  // allow for crystal start-up
    if(RTC_TSR != start) return true;
    delay(10);
  }
  return false;
}

// Measure the 1 kHz LPO that clocks the LPTMR against the main crystal,
// so LPTMR sleeps can be counted in real seconds.
void calibrateLPO(){
  SIM_SCGC5 |= SIM_SCGC5_LPTIMER;
  LPTMR0_CSR = 0;
  LPTMR0_PSR = LPTMR_PSR_PBYP | LPTMR_PSR_PCS(1); // LPO, no prescaler
  LPTMR0_CMR = 499;                               // flag every 500 ticks
  LPTMR0_CSR = LPTMR_CSR_TEN;
  uint32_t flagTime[2];
  for(int n = 0; n < 2; n++){
    uint32_t start = millis();
    while(!(LPTMR0_CSR & LPTMR_CSR_TCF)){
      if(millis() - start > 1000){  // LPO not running: keep the last value
        LPTMR0_CSR = 0;
        return;
      }
    }
    flagTime[n] = micros();
    LPTMR0_CSR = LPTMR_CSR_TEN | LPTMR_CSR_TCF; // clear the flag
  }
  LPTMR0_CSR = 0;
  float hz = 500.0 * 1000000.0 / (float) (flagTime[1] - flagTime[0]);
  if(hz > 500.0 && hz < 2000.0) lpoHz = hz;
  Serial.print("LPO Hz ");
  Serial.println(lpoHz);
}

void setSoftTime(time_t newTime){
  softTime = newTime;
  softMillis = millis();
}

time_t softNow(){
  uint32_t elapsed = millis() - softMillis;
  softTime += elapsed / 1000;
  softMillis += (elapsed / 1000) * 1000;  // keep the fraction of a second
  return softTime;
}

// Switch away from the DS3231 for the rest of the deployment
void useBackupClock(){
  rtcFail = true;
  clockSource = internalRtcOK ? CLOCK_INTERNAL : CLOCK_LPTMR;
  disable_alarm();  // in case the DS3231 still answers, so it cannot hold INT low
  Serial.print("DS3231 failed, clock source ");
  Serial.println(clockSource);
}

// Check the clocks at power-up and pick the best one that works
void clockInit(){
  internalRtcOK = internalRTCRunning();
  calibrateLPO();

  boolean dsOK = readRTC();
  if(dsOK){
    if (t < 1678737100) setTime2(16, 0, 0, 1, 12, 2024 - 2000U);
    // Check DS3231 oscillator status
    if(rtcStatus() & 0x80){ // OSF bit set — oscillator stopped
      // Clear OSF flag and enable oscillator
      reset_alarm(); // clears status register 0x0F (including OSF)

      // Re-write time to kick oscillator
      readRTC();
      setTime2(hour(t), minute(t), second(t), day(t), month(t), year(t) - 2000U);

      cDisplay();
      display.println("DS3231 OSC");
      display.println("STOPPED");
      display.setTextSize(1);
      display.print("Clearing OSF...");
      display.display();
      delay(3000);

      dsOK = ds3231Running();
      cDisplay();
      display.println("DS3231 OSC");
      if(dsOK) display.println("RECOVERED");
      else display.println("FAILED");
      display.display();
      delay(3000);
    }
    else dsOK = ds3231Running();
  }

  if(dsOK){
    clockSource = CLOCK_DS3231;
    t = getTeensy3Time(1);  // sync teensy rtc to DS3231
    return;
  }

  useBackupClock();
  if(internalRtcOK) t = Teensy3Clock.get();
  else{
    if(softTime < 1678737100) setSoftTime(DEFAULT_TIME);
    t = softNow();
  }
  cDisplay();
  display.println("RTC FAIL");
  display.setTextSize(1);
  display.println("Replace DS3231");
  if(internalRtcOK) display.println("Using internal clock");
  else display.println("Using LPO timer");
  display.println("Check date/time");
  display.display();
  delay(5000);
}

// Sleep until wakeTime on whichever clock is working. The LPTMR also wakes the
// logger at least once a minute, so a clock that stops during sleep is caught
// by getTeensy3Time() and the next sleep uses the backup clock.
// sgtlOff: switch the SGTL5000 off while asleep (I2C needs it on).
void sleepUntil(time_t wakeTime, boolean sgtlOff){
  if(clockSource == CLOCK_LPTMR) calibrateLPO();
  while(1){
    long secs = wakeTime - getTeensy3Time(1);
    if(secs <= 0) break;

    // with a clock alarm the LPTMR is a backup 2 s late; on its own it is the alarm
    long lpSecs = (clockSource == CLOCK_LPTMR) ? secs : secs + 2;
    if(lpSecs > 60) lpSecs = 60;
    uint32_t ticks = lpSecs * lpoHz;
    if(ticks > 65000) ticks = 65000;
    lptmr.setTimer(ticks);

    int who;
    if(clockSource == CLOCK_DS3231){
      set_alarm_at(wakeTime);
      if(sgtlOff) digitalWrite(SGTL_EN, LOW);
      who = Snooze.deepSleep(config_ds3231);
    }
    else if(clockSource == CLOCK_INTERNAL){
      rtcAlarm.setRtcTimer(secs / 3600, (secs % 3600) / 60, secs % 60);
      if(sgtlOff) digitalWrite(SGTL_EN, LOW);
      who = Snooze.deepSleep(config_internal);
    }
    else{
      if(sgtlOff) digitalWrite(SGTL_EN, LOW);
      who = Snooze.deepSleep(config_lptmr);
    }
    /// ... sleeping ...
    if(sgtlOff) digitalWrite(SGTL_EN, HIGH); // power on so I2C works

    // Snooze adds the LPTMR ticks to millis() as if they were ms; correct to real time
    if(who == LPTMR_WAKE) softMillis += ticks - (uint32_t) (ticks * 1000.0 / lpoHz);
    if(clockSource == CLOCK_DS3231) reset_alarm();
  }
}
