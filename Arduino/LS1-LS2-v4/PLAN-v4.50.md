# LS1/LS2 v4.50 plan: watchdog, clock and SD card robustness

Status: plan agreed, not yet implemented. Applies to `Arduino/LS1-LS2-v4` and `tools/logger-setup.html`.

## Why

Recent bench and field results:

- A board locked up awake (display dark) after a DS3231M misbehaved. The build was using an old
  Wire library with no timeouts (now moved to `oldLibraries`); Teensyduino's Wire times out and
  frees a stuck bus. A watchdog is still needed to catch hangs we have not found.
- DS3231M parts bought by PCBWay from non-authorized sources stop or fail in the field, with new
  coin cells. The logger must keep recording on the internal RTC when that happens.
- Sleep current with the SanDisk Extreme card is 1.38 mA, but 61 mA with an Amazon Basics card,
  because the card stays powered during sleep. Powering it off should be an option.

## Done first in 4.41 (diagnostic build)

- **Wake on the internal RTC alarm**, not the DS3231 alarm. The DS3231 is the time reference
  only, read and synced at each wake; its alarm is switched off at start-up and INT no longer
  wakes the board. LPTMR backup in the same Snooze block. Reason: a board locked up awake inside
  `sleepUntil()` (94 mA, display dark, LED off, I2C idle) without the DS3231 oscillator stopping.
- DS3231 counts as stuck if its time does not change while the internal RTC advances 5 s (works
  across sleep), as well as the existing 5 s by `millis()` while awake.
- `sleepDiags` (on in 4.41): each sleep and wake printed on USB serial with the wake source, INT
  pin, internal RTC time and the DS3231 time, control and status registers.
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
  per wake, but a loop that keeps waking immediately (seen on the bench 2026-10-29: awake at 94 mA,
  display dark, LED off, I2C idle) adds up to 60 s quickly and triggers it.
- Diagnostic option (`printDiags`): print each wake in `sleepUntil()` with the wake source, clock
  source and seconds left, so a looping sleep can be identified on the bench.
- A watchdog reset during a deployment **resumes** recording, like a software reset: no menu wait,
  no delay days. The resume check accepts RCM_SRS0 WDOG as well as RCM_SRS1 SW. Each watchdog reset
  is counted in EEPROM for the setup page.
- **Turned off** for the manual stop and for the card-full and low-battery stops, so a stopped
  logger stays stopped.

## B. Clock

1. **Proper I2C STOP** in `readRTC()` and `rtcStatus()`; remove the stray `endTransmission()` calls.
2. **DS3231 check against the internal RTC.** The DS3231 stays the reference clock (temperature
   compensated, lower drift). The comparison is only meaningful when **both clocks are ticking**, so
   at each sync:
   - **Each clock must have moved forward since its last read.** If the DS3231 has not, it has
     failed: switch to the internal RTC. If the internal RTC has not, the internal RTC has failed:
     keep the DS3231, stop using the internal RTC as a backup or for comparison, and record it for
     the setup page.
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

## C. SD card

1. **Shared card routines:** `cardOn()` powers up, starts the card with 5 power-cycle retries and
   returns to the current folder; `cardOff()` waits until the card is idle, releases the SPI pins,
   holds CS low and cuts power. Used at power-up, after the delayed start, and by SD Sleep.
2. **New menu setting "SD Sleep"**, default `Keep powered`; the other value is `Power off`. The
   setting's screen shows "Sleeps 2 min or more". When set to `Power off`, the card is turned off
   for sleeps of 2 minutes or more and started again during the 4 s wake-up margin. If it will not
   start, the logger restarts and resumes as now. Help line: `ENTER:SD in long sleep`.
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

## E. Display start-up

Some boards occasionally power up with the display dark while the firmware runs normally (green
LED on, normal serial start-up). Seen before 4.40 as well. The display has no reset line, so it relies on its own
power-on reset, which can end before a slowly rising supply settles; it then ignores the single
`display.begin()` in `setup()`.

- On power-on only (RCM_SRS0 POR set), call `display.begin()` a second time about 150 ms after
  the first. Software restarts and the re-inits after each wake and the manual stop stay as now.

## F. Version

Bump to **4.50**.

## New EEPROM bytes

| Byte | Use |
|---|---|
| 29 | SD Sleep setting (0 = keep powered, 1 = power off) |
| 30 | Watchdog reset count (stops at 254) |
| 31 | Clock switch flag |
| 32-35 | Time of the clock switch (seconds since 1970) |

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
- **Coin cell:** fit a coin cell on bench boards before judging DS3231 stop-flag results.
