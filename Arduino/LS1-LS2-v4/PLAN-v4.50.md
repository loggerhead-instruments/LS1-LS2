# LS1/LS2 v4.50 plan: watchdog, clock and SD card robustness

Status: plan agreed, not yet implemented. Applies to `Arduino/LS1-LS2-v4` and `tools/logger-setup.html`.

## Why

Recent bench and field results:

- Boards stuck awake after a wake (display dark, LED off; 94 mA with an Amazon Basics card, 40 mA
  with a SanDisk). First blamed on a DS3231M and the old Wire library, then on a loop inside
  `sleepUntil()`. It was the audio stall fixed in 4.42: the logger had woken, opened the next file
  and was waiting in record mode for audio that never came (the display is always off while
  recording). The old Wire library, with no timeouts, was still worth removing (now in
  `oldLibraries`). A watchdog is still needed to catch hangs we have not found.
- DS3231M parts bought by PCBWay from non-authorized sources stop or fail in the field, with new
  coin cells. The logger must keep recording on the internal RTC when that happens.
- Sleep current with the SanDisk Extreme card is 1.38 mA, but 61 mA with an Amazon Basics card,
  because the card stays powered during sleep. Powering it off should be an option.

## Done first in 4.41 (diagnostic build)

- **Wake on the internal RTC alarm**, not the DS3231 alarm. The DS3231 is the time reference
  only, read and synced at each wake; its alarm is switched off at start-up and INT no longer
  wakes the board. LPTMR backup in the same Snooze block. Reason at the time: a board stuck awake, thought to be
  inside `sleepUntil()`; it was later found to be the 4.42 audio stall. Waking on the internal RTC
  is kept because it does not depend on the DS3231.
- DS3231 counts as stuck if its time does not change while the internal RTC advances 5 s (works
  across sleep), as well as the existing 5 s by `millis()` while awake.
- `sleepDiags` (on in 4.41): each sleep and wake printed on USB serial with the wake source, INT
  pin, internal RTC time and the DS3231 time, control and status registers. (These were lost while
  USB reconnected; 4.42 stores them and prints them at Record Start.)
- Proper I2C STOP in `readRTC()` and `rtcStatus()` (review problem 1).

## Done in 4.42 (bench build)

- **Audio stops properly before deep sleep.** Bench boards sometimes woke, opened the next file and
  then got no audio (40 mA, no LED, stuck after `Queue Begin`). The 2024 code noted the same
  fault ("no LRCLK signal or RX on Teensy"). Since 4.10 the deep sleep stops all fast clocks while
  the I2S transmitter was still running. Now `audioSleep()` powers the codec down over I2C, then
  `i2sStop()` stops the I2S receiver and transmitter at a frame end, resets the FIFOs, clears the
  error flags and parks the receive DMA. On waking, `I2S_modification()` sets the clocks only while
  the I2S is stopped and `i2sStart()` restarts it as the audio library does, before the codec is
  powered up. Where the codec power is cut (delayed start, stops), the I2S clock pins are driven
  low first so they do not power the codec through its pins.
- `diag.ino`: wake log printed at Record Start, and a no-audio report (serial and display).
- Bench result: rec 60 / sleep 120 on the internal clock, no stall over more than an hour (4.41
  stalled on the third wake). Overnight run in progress.

## Problems found in review

1. `readRTC()` and `rtcStatus()` never send an I2C STOP and call a stray `endTransmission()`,
   leaving the bus half-finished going into every sleep.
2. `FileInit()` fallback names (`F000001.wav`) open the file twice, so one failed open always
   ends in `resetFunc()` after 20 tries, even when the card works. In continuous mode it also
   calls `chdir()` with an empty folder name.
3. After a delayed start, `sd.begin()` is called once with no check or retry, and the card power
   is cut without releasing the SPI pins first.
4. A dead card restarts forever: about every 40 s at about 40 mA until the batteries are flat.
5. The UP+DOWN manual stop waits in `while(1)`. Once a watchdog exists, that would reset the board
   and start a new deployment after the menu timeout.

## A. Hardware watchdog

- Kinetis WDOG clocked by the 1 kHz LPO, **60 s timeout**, enabled at the start of `setup()`.
  The Teensy core leaves it updatable (ALLOWUPDATE), so it can be enabled there.
- **Paused in deep sleep** (STOPEN off; LLS is a stop mode), running during `wfi`. A sleep of any
  length cannot trigger it. It is refreshed right after each wake.
- Refreshed in `loop()`, the menu loop, in each card retry, and in `resetFunc()`.
  **Not refreshed inside the `sleepUntil()` loop**, only before and after the whole sleep. The
  watchdog is paused during real sleep, so a normal long sleep adds only milliseconds of awake time
  per wake, but a loop that keeps waking immediately adds up to 60 s quickly and triggers it.
- Diagnostic option (`printDiags`): print each wake in `sleepUntil()` with the wake source, clock
  source and seconds left, so a looping sleep can be identified on the bench.
- A watchdog reset during a deployment **resumes** recording, like a software reset: no menu wait,
  no delay days. The resume check accepts RCM_SRS0 WDOG as well as RCM_SRS1 SW. Each watchdog reset
  is counted in EEPROM for the setup page.
- **Turned off** for the manual stop and for the card-full and low-battery stops, so a stopped
  logger stays stopped.
- **Audio watchdog** (from the 4.42 no-audio check, always on): if no audio reaches the card for
  3 s during a recording, run `AudioInit()` again and count it. If there is still no audio after
  another 3 s, restart the logger (`resetFunc()`), which resumes the deployment. The count is shown
  on the setup page. The rest of `diag.ino` stays a bench option.

## B. Clock

1. **Proper I2C STOP** in `readRTC()` and `rtcStatus()`; remove the stray `endTransmission()` calls.
2. **DS3231 check against the internal RTC.** The DS3231 stays the reference clock (temperature
   compensated, lower drift). The comparison is only meaningful when **both clocks are ticking**, so
   at each sync:
   - **Each clock must have moved forward since its last read.** If the DS3231 has not, it has
     failed: switch to the internal RTC. If the internal RTC has not, the internal RTC has failed:
     keep the DS3231, stop using the internal RTC as a backup or for comparison, and record it for
     the setup page. Sleeps then use the LPTMR alone, as `sleepUntil()` already does when the
     internal RTC is not running: wake at least every 60 s, read the DS3231, and sleep the
     remaining seconds. The LPTMR runs on the LPO, a 1 kHz RC oscillator inside the chip that keeps
     running in deep sleep. Before each sleep it is calibrated against `micros()`, which comes from
     the main processor crystal (running whenever the processor is awake), not the 32 kHz RTC
     crystal, so the last step is well under 1 s off even when the 32 kHz crystal has failed.
     The DS3231 INT is not used to wake: a failing DS3231M could leave INT high (late wake) or hold
     it low (repeated wakes). If the internal RTC stops during a sleep, the LPTMR backup in the same
     sleep still wakes the logger within 60 s.
   - **DS3231 ticking and its year is 2026 or later:** the DS3231 is correct. Sync the internal RTC
     to it as now. If the two were more than 10 s apart, count the event for the setup page.
   - **DS3231 not ticking, or its year is before 2026:** the DS3231 has failed (stopped, or lost its
     time). Switch to the internal RTC and do not copy the bad time to it.
   - The existing 5 s frozen check while awake stays. Setting the time from the menu or setup page
     sets both clocks.
3. **Resume with the stop flag (OSF) set:** use the internal RTC, which kept counting through the
   reset, for the rest of the deployment, and write its time back to the DS3231.
4. **Record a clock switch** in EEPROM with the time it happened. Show "Clk: int" on the
   "Next Start" screen. The setup page reports "DS3231 failed during deployment at ..." until cleared.
5. **Correct `millis()` after an RTC alarm wake.** Snooze's LPTMR driver adds its whole period to
   `millis()` on every wake, including wakes by the RTC alarm before the LPTMR period has run out,
   so `millis()` gains about 2 s per sleep. After a `35` wake, subtract the part of the LPTMR period
   that did not pass (read from the LPTMR counter). This matters for the LPTMR-only clock and for
   `millis()` timeouts that span a sleep.

## C. SD card

1. **Shared card routines:** `cardOn()` powers up, starts the card with 5 power-cycle retries and
   returns to the current folder; `cardOff()` waits until the card is idle, releases the SPI pins,
   holds CS low and cuts power. Used at power-up, after the delayed start, and by SD Sleep.
   The April 2026 attempt (162a0f1, removed in a2e3ebe) failed because it only wrote the SPI pins
   low with `digitalWrite()` while they were still assigned to SPI, so MOSI and SCK kept driving
   the unpowered card through its protection diodes (about 2.67 V) and the current did not drop.
   That commit blamed VLLS losing pin states, but the logger uses LLS, which keeps them. The pins
   must be taken off SPI (`pinMode(pin, INPUT_DISABLE)`, as `stopForGood()` does) before the power
   is cut.
2. **New menu setting "SD Sleep"**, default `Keep powered`; the other value is `Power off`. The
   setting's screen shows "Sleeps 2 min or more". When set to `Power off`, the card is turned off
   for sleeps of 2 minutes or more and started again during the 4 s wake-up margin. If it will not
   start, the logger restarts and resumes as now. Help line: `ENTER:SD power down`.
3. **Fix the `FileInit()` fallback** so one bad open does not force a restart; restart only if a
   fallback name also fails. No `chdir()` with an empty folder name.
4. **Back off after repeated card failures:** after 5 failed restarts in a row, sleep 1 hour with
   the card off, then try again, for as long as the batteries last. The counter is kept in a chip
   register that survives restarts (not EEPROM), so no wear.
## D. Setup page (`tools/logger-setup.html`)

- SD Sleep setting: "microSD card during sleeps of 2 min or more": `Keep powered (default)` or
  `Power off (saves current; card restarts each wake)`.
- Watchdog reset count, with Clear.
- Clock-switch report with the time, with Clear.

## E. Start-up

### Codec and hydrophone power cycle on every start

`SGTL_EN` (pin 6) and the hydrophone enable have no pull-up or pull-down. During a reset and the
first 0.5 s of `setup()` the Teensy pins are inputs, so both enables float and the codec and
hydrophone may be on, off or in between. Only the SD card is power-cycled for certain
(`resetFunc()` cuts it, `setup()` holds `SDPOW1` low).

- At the very start of `setup()`, before the 500 ms serial wait, drive `SGTL_EN` and the hydrophone
  enable low and the I2S clock pins low (as `i2sPinsLow()` does), hold for 100 ms, then switch both
  on. The codec gets a real power cycle on every start, including restarts after a card failure and
  watchdog resets, and the enables no longer float during the serial wait. `AudioInit()` then sets
  up the I2S and the codec as now.
- Hardware, next board revision: add pull-downs (for example 100 k) on `SGTL_EN` and the hydrophone
  enable, so they are defined during reset too.

### Display

Some boards occasionally power up with the display dark while the firmware runs normally (green
LED on, normal serial start-up). Seen before 4.40 as well. The display has no reset line, so it relies on its own
power-on reset, which can end before a slowly rising supply settles; it then ignores the single
`display.begin()` in `setup()`.

- On power-on only (RCM_SRS0 POR set), call `display.begin()` a second time about 150 ms after
  the first. Software restarts and the re-inits after each wake and the manual stop stay as now.

## F. Version

Bump to **4.50**, with `sleepDiags = 0`.

## New EEPROM bytes

| Byte | Use |
|---|---|
| 29 | SD Sleep setting (0 = keep powered, 1 = power off) |
| 30 | Watchdog reset count (stops at 254) |
| 31 | Clock switch flag |
| 32-35 | Time of the clock switch (seconds since 1970) |
| 36 | Audio restart count (audio watchdog, stops at 254) |

Existing: 0-21 settings, 24 voltage cutoff, 25 SD restarts, 26 low-battery stop, 27 card-full
stop, 28 deployment running.

## Not doing

- Changing the SdFat library.
- Deeper VLLS sleep (each wake would be a full restart and card start).

## Bench tests

- **Watchdog:** a temporary build that hangs on purpose resets and resumes. After a run with
  sleeps over 60 s, the watchdog reset count is still 0.
- **Clock failure:** pull the DS3231's power or short SDA during a sleep. The logger switches to the
  internal RTC, keeps cycling, and shows the switch on the screen and setup page.
- **SD Sleep:** with `Power off`, sleep current drops to the board's floor (about 1.4 mA or less)
  even with a high-idle card, and files continue in the same folder.
- **Card failures:** pull the card during a deployment; the logger restarts and resumes. A dead
  card reaches the hourly back-off after 5 failures.
- **Display start-up:** power the board on and off 20 times; the display shows the splash screen
  every time.
- **Start-up power cycle:** on a scope, `SGTL_EN` and the hydrophone enable go low for 100 ms at
  every start (power-on, restart after a card failure, watchdog reset), and recording is normal after.
- **Audio:** overnight duty-cycle run with no stall (on USB and on battery). Audio watchdog: a
  temporary build that skips `i2sStart()` on one wake recovers by `AudioInit()` and records.
- **Coin cell:** fit a coin cell on bench boards before judging DS3231 stop-flag results.
