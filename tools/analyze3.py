"""
Analysis script for Dr. Strangelove drift telemetry.

Usage:
    python analyze.py <csv_file>
    python analyze.py            # processes all *.csv in current directory

Outputs (saved next to each input CSV):
    <basename>.png   -- diagnostic plot (5 panels)
    <basename>.txt   -- summary statistics including detected PSD peaks
Also prints summary to terminal.
"""
import argparse
import glob
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal


FS_HZ = 10.0  # must match firmware sample rate
N_PEAKS_TO_ANNOTATE = 5  # how many top peaks to mark and report


def find_top_peaks(freqs, psd, n_peaks=5, min_freq=0.005):
    """Find the n highest-prominence peaks in a PSD.
    Ignores DC and very low frequencies (below min_freq Hz).
    Returns sorted list of (freq_hz, psd_value) tuples, highest PSD first."""
    # Mask out very low frequencies (DC drift dominates and isn't a "peak")
    mask = freqs >= min_freq
    f = freqs[mask]
    p = psd[mask]

    if len(p) < 5:
        return []

    # scipy peak finder. Prominence threshold relative to median PSD.
    median = np.median(p)
    prominence_threshold = median * 2.0
    peak_idx, props = signal.find_peaks(p, prominence=prominence_threshold)

    if len(peak_idx) == 0:
        return []

    # Sort by PSD value at peak (highest first), take top n
    peak_vals = p[peak_idx]
    order = np.argsort(peak_vals)[::-1][:n_peaks]
    peaks = [(float(f[peak_idx[i]]), float(p[peak_idx[i]])) for i in order]
    return peaks


def analyze_one(csv_path):
    """Analyze a single CSV file. Returns True on success, False on failure."""
    print(f"\n=== Processing: {csv_path} ===")

    try:
        df = pd.read_csv(csv_path, comment='#')
    except Exception as e:
        print(f"  ERROR reading file: {e}")
        return False

    required_cols = {'timestamp_us', 'sample', 'errorX', 'errorY', 'totalPower'}
    missing = required_cols - set(df.columns)
    if missing:
        print(f"  ERROR: missing required columns: {missing}")
        return False

    if len(df) < 100:
        print(f"  WARNING: only {len(df)} samples — analysis may be unreliable")

    # --- Time axis ---
    t = df['t_sec']
    t_min = t / 60.0

    # --- Sample-gap detection ---
    sample_diff = df['sample'].diff().dropna()
    n_gaps = int((sample_diff != 1).sum())
    if n_gaps > 0:
        gap_pct = 100.0 * n_gaps / len(sample_diff)
        print(f"  WARNING: {n_gaps} sample gaps detected ({gap_pct:.2f}% of intervals)")
        print(f"           PSD assumes uniform sampling — interpret spectrum with caution")

    nyquist = FS_HZ / 2.0

    # --- Build the figure: 5 panels now (PSD split into log and linear views) ---
    fig, axes = plt.subplots(5, 1, figsize=(11, 13))

    # Panel 1: error time series
    ax = axes[0]
    ax.plot(t_min, df['errorX'], label='errorX', linewidth=0.7, alpha=0.85)
    ax.plot(t_min, df['errorY'], label='errorY', linewidth=0.7, alpha=0.85)
    ax.axhline(0, color='k', linewidth=0.4, alpha=0.4)
    ax.set_xlabel('Time (min)')
    ax.set_ylabel('Normalized error')
    ax.set_title(f'Beam position error vs. time  —  {os.path.basename(csv_path)}')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # Panel 2: totalPower
    ax = axes[1]
    pwr_mean = df['totalPower'].mean()
    ax.plot(t_min, df['totalPower'], color='tab:green', linewidth=0.7)
    ax.axhline(pwr_mean, color='k', linewidth=0.5, linestyle='--', alpha=0.5,
               label=f'mean = {pwr_mean:.0f}')
    ax.set_xlabel('Time (min)')
    ax.set_ylabel('totalPower (ADC counts)')
    ax.set_title('Total power — check for power-induced contamination of error signal')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    # --- PSD computation (shared across panels 3 & 4) ---
    nperseg = min(int(5 * 60 * FS_HZ), len(df) // 4)
    peaks_x, peaks_y = [], []
    have_psd = nperseg >= 32

    if have_psd:
        ex = (df['errorX'] - df['errorX'].mean()).to_numpy()
        ey = (df['errorY'] - df['errorY'].mean()).to_numpy()
        f_x, psd_x = signal.welch(ex, fs=FS_HZ, nperseg=nperseg)
        f_y, psd_y = signal.welch(ey, fs=FS_HZ, nperseg=nperseg)
        peaks_x = find_top_peaks(f_x, psd_x, N_PEAKS_TO_ANNOTATE)
        peaks_y = find_top_peaks(f_y, psd_y, N_PEAKS_TO_ANNOTATE)

    # Panel 3: PSD log-log (broad view, good for finding drift power-law slopes)
    ax = axes[2]
    if have_psd:
        ax.loglog(f_x, psd_x, label='errorX', linewidth=1)
        ax.loglog(f_y, psd_y, label='errorY', linewidth=1)
        ax.axvline(nyquist, color='red', linestyle=':', linewidth=0.8, alpha=0.7,
                   label=f'Nyquist = {nyquist:.1f} Hz')
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (unit²/Hz)')
        ax.set_title('PSD (log-log) — overall spectrum shape and drift behavior')
        ax.legend(loc='lower left', fontsize=8)
        ax.grid(True, which='both', alpha=0.3)
    else:
        ax.text(0.5, 0.5, 'PSD skipped — dataset too short',
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('PSD (log-log)')

    # Panel 4: PSD linear-frequency, log-Y (easier to read peak frequencies)
    ax = axes[3]
    if have_psd:
        ax.semilogy(f_x, psd_x, label='errorX', linewidth=1, alpha=0.8)
        ax.semilogy(f_y, psd_y, label='errorY', linewidth=1, alpha=0.8)

        # Annotate detected peaks. Stagger labels vertically so they don't overlap.
        all_peaks = [('X', f, p) for f, p in peaks_x] + [('Y', f, p) for f, p in peaks_y]
        # Sort by frequency so labels go left-to-right
        all_peaks.sort(key=lambda x: x[1])
        for i, (axis, freq, pval) in enumerate(all_peaks):
            color = 'tab:blue' if axis == 'X' else 'tab:orange'
            ax.axvline(freq, color=color, linestyle='--', linewidth=0.5, alpha=0.5)
            # Stagger y position for labels
            y_offset = 1.5 + (i % 3) * 1.2
            ax.annotate(f'{axis}: {freq:.2f} Hz',
                        xy=(freq, pval),
                        xytext=(freq, pval * y_offset),
                        fontsize=8, color=color,
                        ha='center',
                        arrowprops=dict(arrowstyle='-', color=color, alpha=0.4, lw=0.5))

        ax.axvline(nyquist, color='red', linestyle=':', linewidth=0.8, alpha=0.7)
        ax.set_xlim(0, nyquist * 1.02)
        ax.set_xlabel('Frequency (Hz)')
        ax.set_ylabel('PSD (unit²/Hz)')
        ax.set_title('PSD (linear frequency) — peak frequencies annotated')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
    else:
        ax.text(0.5, 0.5, 'PSD skipped — dataset too short',
                ha='center', va='center', transform=ax.transAxes)
        ax.set_title('PSD (linear)')

    # Panel 5: histograms
    ax = axes[4]
    ax.hist(df['errorX'], bins=80, alpha=0.6, label=f'errorX (σ={df["errorX"].std():.4f})')
    ax.hist(df['errorY'], bins=80, alpha=0.6, label=f'errorY (σ={df["errorY"].std():.4f})')
    ax.set_xlabel('Normalized error')
    ax.set_ylabel('Count')
    ax.set_title('Error distribution — informs deadband sizing')
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    plt.tight_layout()

    # --- Sibling output paths ---
    base, _ = os.path.splitext(csv_path)
    png_path = base + '.png'
    txt_path = base + '.txt'

    plt.savefig(png_path, dpi=120, bbox_inches='tight')
    plt.close(fig)
    print(f"  Wrote {png_path}")

    # --- Summary stats ---
    first_min = df[t_min < 1]
    last_min = df[t_min > t_min.iloc[-1] - 1]

    lines = []
    lines.append(f"Source:              {csv_path}")
    lines.append(f"Samples:             {len(df)}")
    lines.append(f"Run duration:        {t.iloc[-1]/60:.2f} min")
    lines.append(f"Sample rate (assumed): {FS_HZ} Hz  (Nyquist = {nyquist:.1f} Hz)")
    if n_gaps > 0:
        lines.append(f"Sample gaps:         {n_gaps} ({100.0*n_gaps/len(sample_diff):.2f}% of intervals)")
    else:
        lines.append(f"Sample gaps:         none")
    lines.append("")
    lines.append(f"errorX:  mean={df['errorX'].mean():+.4f}  std={df['errorX'].std():.4f}  "
                 f"min={df['errorX'].min():+.4f}  max={df['errorX'].max():+.4f}")
    lines.append(f"errorY:  mean={df['errorY'].mean():+.4f}  std={df['errorY'].std():.4f}  "
                 f"min={df['errorY'].min():+.4f}  max={df['errorY'].max():+.4f}")
    lines.append(f"totalPower: mean={df['totalPower'].mean():.0f}  "
                 f"std={df['totalPower'].std():.0f}  "
                 f"({100*df['totalPower'].std()/df['totalPower'].mean():.2f}% RMS)")
    lines.append("")
    lines.append("Net drift over run (last-minute mean − first-minute mean):")
    lines.append(f"  errorX: {last_min['errorX'].mean() - first_min['errorX'].mean():+.4f}")
    lines.append(f"  errorY: {last_min['errorY'].mean() - first_min['errorY'].mean():+.4f}")
    lines.append("")
    lines.append("Detected PSD peaks (sorted by frequency):")
    if not peaks_x and not peaks_y:
        lines.append("  none above prominence threshold")
    else:
        all_peaks = ([('errorX', f, p) for f, p in peaks_x]
                     + [('errorY', f, p) for f, p in peaks_y])
        all_peaks.sort(key=lambda x: x[1])
        for axis, freq, pval in all_peaks:
            lines.append(f"  {axis}: f = {freq:7.3f} Hz   PSD = {pval:.3e}")
        lines.append("")
        lines.append("Note: at fs=10 Hz, the Nyquist limit is 5 Hz. Real-world disturbances")
        lines.append("above 5 Hz alias into the 0–5 Hz band — observed peak frequencies may")
        lines.append("correspond to higher-frequency physical sources.")

    summary = '\n'.join(lines)
    with open(txt_path, 'w') as f:
        f.write(summary + '\n')
    print(f"  Wrote {txt_path}")
    print()
    print(summary)
    return True


def main():
    parser = argparse.ArgumentParser(
        description='Analyze Dr. Strangelove drift telemetry CSV files.')
    parser.add_argument('csv', nargs='*',
                        help='CSV file(s) to analyze. If omitted, processes all *.csv '
                             'in the current directory.')
    args = parser.parse_args()

    if args.csv:
        paths = args.csv
    else:
        paths = sorted(glob.glob('*.csv'))
        if not paths:
            print("No CSV files specified and none found in current directory.")
            print("Usage: python analyze.py [file1.csv file2.csv ...]")
            sys.exit(1)
        print(f"Found {len(paths)} CSV file(s) in current directory.")

    ok = 0
    for p in paths:
        if analyze_one(p):
            ok += 1
    print(f"\n=== Done. {ok}/{len(paths)} processed successfully. ===")


if __name__ == '__main__':
    main()
