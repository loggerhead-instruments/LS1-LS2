# SDWriteTest log

## Run 1: SdFat-beta 2.0.0-beta.7, one month folder (failed)

- 1 TB card, exFAT, 512 KB clusters. 48 kHz mono, 300 s files (28.8 MB), all in `/2026-11`.
- 47 GB: open 327 ms, longest write 1172 ms.
- 56.5 GB: open 393 ms, longest write 131 ms, close 2004 ms.
- Hung while starting file 1961 (in the open or the header write), after file 1960's
  2004 ms close had completed. Green LED off, display unchanged for 4 hours, no Card Fail.
- On a PC: 1961 files in `/2026-11`, the last one 0 KB. chkdsk found file 1961's entry set
  half written.
- This build had no watchdog, so it stayed stuck. The test was then rebuilt with the
  watchdog, the logger's restart code and SdFat 2.1.2.

## Run 2: SdFat 2.1.2 (src/SdFat), day folders, watchdog

- Same 1 TB card, files deleted, not reformatted. 300 kHz mono, 300 s files (180 MB),
  fast fill first.
- Open time is about 0.2 ms per file already in the folder, the same rate as run 1. Day
  folders hold at most 288 files, so opens stay under about 60 ms.

| Date | Filled | Longest write | Writes >100 ms | Writes >250 ms | W O C R WD | Longest open | Longest close |
|---|---|---|---|---|---|---|---|
| 2026-11-01 | 291 GB | 258 ms | 512 | 8 | 0 0 0 0 0 | 60 ms | 21 ms |
| 2026-11-05 | 62% | 258 ms | | | 0 0 0 0 0 | 67 ms | 40 ms |

- 2026-11-05: still running. The longest write is still 258 ms, well inside SdFat's 600 ms
  write timeout. At 62% of the card the run has written far more files than run 1's 1961,
  so it has passed the number of opens and closes at which run 1 hung. It has not tested a
  folder with as many files as run 1's (about 1960).
- 2026-11-05: no errors. The display shows dropped samples. At 300 kHz the audio buffer
  holds only about 43 ms, so any longer write drops audio. To be tested separately.
