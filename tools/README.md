# Drift Telemetry Analysis

`analyze.py` processes CSV files produced by the Dr. Strangelove firmware
telemetry mode and generates diagnostic plots and summary statistics for
each run.

## Setup

```
pip install -r requirements.txt
```

Python 3.8+ recommended. A virtual environment is suggested but not required.

## Usage

Analyze a single CSV file:
```
python analyze.py strangelove_20260618_121044.csv
```

Analyze multiple files:
```
python analyze.py run1.csv run2.csv run3.csv
```

Analyze every `*.csv` in the current directory:
```
python analyze.py
```

## Outputs

For each input CSV, the script writes two sibling files with the same basename:

- `<basename>.png` — five-panel diagnostic plot:
  1. errorX / errorY vs. time
  2. totalPower vs. time
  3. PSD log-log (overall spectrum shape and drift behavior)
  4. PSD linear-frequency, log-Y (peak frequencies annotated)
  5. Error distribution histogram
- `<basename>.txt` — summary statistics, sample-gap report, and detected PSD peaks

A summary is also printed to the terminal as each file is processed.

## Expected CSV format

The script expects CSV files produced by the firmware in telemetry mode. The
following columns are required:

- `timestamp_us` — Teensy `micros()` timestamp
- `sample` — monotonic sample counter (used for gap detection)
- `errorX`, `errorY` — normalized position errors
- `totalPower` — sum of all four quadrants

Header lines starting with `#` (run metadata) are ignored.

## Notes

- **Sample rate is hardcoded** to `FS_HZ = 10.0` at the top of the script. If
  the firmware sample rate changes, update this constant. A mismatch silently
  produces incorrect frequency-axis values in the PSD plots.
- **At fs = 10 Hz the Nyquist limit is 5 Hz.** Real-world disturbances above
  5 Hz alias into the 0–5 Hz band — observed peak frequencies may correspond
  to higher-frequency physical sources.
- **Sample-gap detection** flags any missing samples (e.g. dropped USB
  packets). The PSD assumes uniform sampling; gaps will distort the spectrum.
- **Peak detection** uses a prominence threshold of 2× the median PSD value
  and reports the top 5 peaks per axis. Tune `N_PEAKS_TO_ANNOTATE` or the
  prominence threshold in `find_top_peaks()` if needed.
