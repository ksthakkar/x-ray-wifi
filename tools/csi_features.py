"""
Feature bank for CSI presence detection — a broad sweep, not a guess.

The point of this module is to avoid the mistake of assuming which statistic
should carry the signal. An earlier version of the analysis used only
fluctuation features (variance, mean-absolute-difference, band power) and
concluded there was no signal, while a hand on the board was in fact collapsing
individual subcarriers to ~10% of their empty-room amplitude. Fluctuation
features measure how much a subcarrier WOBBLES; that hand produced a large,
nearly static change in the channel's SHAPE. Wrong question, wrong answer.

So: enumerate features across every axis a body could plausibly affect, and let
the empty-vs-empty control decide which ones are real rather than intuition.

Axes covered
------------
1.  Level / shape    — mean amplitude, per-subcarrier profile, profile distance,
                       spectral shape (tilt, curvature), min/max, nulls
2.  Fluctuation      — variance, std, MAD, first-difference stats, range, IQR
3.  Distribution     — skew, kurtosis, entropy, quantiles, coefficient of var.
4.  Temporal/spectral— band power in several bands, spectral centroid/flatness,
                       autocorrelation at multiple lags, zero crossings
5.  Cross-subcarrier — correlation structure, effective rank, subcarrier
                       coherence, PCA-like dominant-direction energy
6.  Stability        — drift within segment, split-half distance, novelty vs a
                       reference profile
7.  Link-layer       — RSSI stats, frame rate, inter-frame timing (a body can
                       change packet reception, not just the channel)

Every feature returns EITHER a per-subcarrier vector (analysed per subcarrier
and aggregated) or a single scalar per segment. Both are handled by the caller.
"""

import math

# ------------------------------------------------------------------ helpers


def mean(xs):
    return sum(xs) / len(xs) if xs else 0.0


def variance(xs):
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    return sum((x - m) ** 2 for x in xs) / (len(xs) - 1)


def stdev(xs):
    return math.sqrt(variance(xs))


def median(xs):
    if not xs:
        return 0.0
    s = sorted(xs)
    n = len(s)
    return s[n // 2] if n % 2 else 0.5 * (s[n // 2 - 1] + s[n // 2])


def quantile(xs, q):
    if not xs:
        return 0.0
    s = sorted(xs)
    i = q * (len(s) - 1)
    lo = int(math.floor(i))
    hi = min(lo + 1, len(s) - 1)
    return s[lo] + (s[hi] - s[lo]) * (i - lo)


def skewness(xs):
    if len(xs) < 3:
        return 0.0
    m, sd = mean(xs), stdev(xs)
    if sd == 0:
        return 0.0
    return sum(((x - m) / sd) ** 3 for x in xs) / len(xs)


def kurtosis(xs):
    """Excess kurtosis (0 for a Gaussian)."""
    if len(xs) < 4:
        return 0.0
    m, sd = mean(xs), stdev(xs)
    if sd == 0:
        return 0.0
    return sum(((x - m) / sd) ** 4 for x in xs) / len(xs) - 3.0


def entropy(xs, bins=16):
    """Shannon entropy of the value histogram, in nats.

    Sensitive to how 'spread out' a distribution is in a way variance is not:
    a bimodal series (person moves between two positions) can have the same
    variance as a unimodal one but higher entropy.
    """
    if len(xs) < bins:
        return 0.0
    lo, hi = min(xs), max(xs)
    if hi <= lo:
        return 0.0
    counts = [0] * bins
    for x in xs:
        k = int((x - lo) / (hi - lo) * (bins - 1))
        counts[k] += 1
    n = len(xs)
    return -sum((c / n) * math.log(c / n) for c in counts if c)


def pearson(a, b):
    if len(a) < 2 or len(a) != len(b):
        return 0.0
    ma, mb = mean(a), mean(b)
    num = sum((x - ma) * (y - mb) for x, y in zip(a, b))
    da = math.sqrt(sum((x - ma) ** 2 for x in a))
    db = math.sqrt(sum((y - mb) ** 2 for y in b))
    return num / (da * db) if da > 0 and db > 0 else 0.0


def autocorr(xs, lag):
    if len(xs) <= lag + 1:
        return 0.0
    return pearson(xs[:-lag], xs[lag:])


def zero_crossings(xs):
    """Crossings of the mean — a crude frequency proxy that needs no FFT."""
    if len(xs) < 2:
        return 0.0
    m = mean(xs)
    c = 0
    for i in range(len(xs) - 1):
        if (xs[i] - m) * (xs[i + 1] - m) < 0:
            c += 1
    return c / (len(xs) - 1)


# -------------------------------------------------------------- resampling


def resample_uniform(ts_us, vals, min_len=32):
    """Resample onto a uniform grid using the real timestamps.

    The CSI rate is variable (packets arrive when they arrive), so any spectral
    measure computed against a nominal rate would smear. Returns (grid, fs) or
    (None, 0) if the segment is too short to be meaningful.
    """
    if len(vals) < min_len:
        return None, 0.0
    span = (ts_us[-1] - ts_us[0]) / 1e6
    if span <= 0:
        return None, 0.0
    fs = len(vals) / span
    n = int(span * fs)
    if n < min_len:
        return None, 0.0
    grid = []
    j = 0
    for k in range(n):
        target = ts_us[0] + k / fs * 1e6
        while j + 1 < len(ts_us) - 1 and ts_us[j + 1] < target:
            j += 1
        grid.append(vals[j])
    return grid, fs


def detrend(xs):
    """Remove mean and linear trend, so slow drift does not leak into low bins."""
    n = len(xs)
    if n < 3:
        return list(xs)
    m = mean(xs)
    g = [v - m for v in xs]
    xm = (n - 1) / 2
    sxy = sum((i - xm) * g[i] for i in range(n))
    sxx = sum((i - xm) ** 2 for i in range(n))
    slope = sxy / sxx if sxx else 0.0
    return [g[i] - slope * (i - xm) for i in range(n)]


def hann(xs):
    n = len(xs)
    if n < 2:
        return list(xs)
    return [xs[i] * 0.5 * (1 - math.cos(2 * math.pi * i / (n - 1))) for i in range(n)]


def power_spectrum(grid, fs, max_bins=128):
    """Periodogram via direct DFT. Returns [(freq, power)].

    Direct DFT rather than FFT to stay dependency-free; bins are capped so the
    O(n*k) cost stays bounded on long segments.
    """
    g = hann(detrend(grid))
    n = len(g)
    if n < 8:
        return []
    kmax = min(n // 2, max_bins)
    out = []
    for k in range(1, kmax):
        w = 2 * math.pi * k / n
        re = im = 0.0
        for i in range(n):
            re += g[i] * math.cos(w * i)
            im -= g[i] * math.sin(w * i)
        out.append((k * fs / n, (re * re + im * im) / (n * n)))
    return out


def band_power(spec, f_lo, f_hi):
    return sum(p for f, p in spec if f_lo <= f < f_hi)


def spectral_centroid(spec):
    tot = sum(p for _f, p in spec)
    return sum(f * p for f, p in spec) / tot if tot > 0 else 0.0


def spectral_flatness(spec):
    """Geometric/arithmetic mean ratio. ~1 = noise-like, ~0 = tonal.

    A breathing rhythm is tonal; ambient channel noise is flat. This
    distinguishes them without needing to know the exact frequency.
    """
    ps = [p for _f, p in spec if p > 0]
    if len(ps) < 2:
        return 0.0
    log_mean = sum(math.log(p) for p in ps) / len(ps)
    return math.exp(log_mean) / mean(ps) if mean(ps) > 0 else 0.0


# ------------------------------------------------- per-subcarrier features
# Each takes (series, ts_us) for ONE subcarrier and returns a scalar.

def _spec_of(series, ts_us, cache):
    if "spec" not in cache:
        grid, fs = resample_uniform(ts_us, series)
        cache["spec"] = power_spectrum(grid, fs) if grid else []
    return cache["spec"]


PER_SC = {}


def per_sc(name):
    def deco(fn):
        PER_SC[name] = fn
        return fn
    return deco


# --- level / shape
@per_sc("mean")
def f_mean(s, t, c):
    return mean(s)


@per_sc("median")
def f_median(s, t, c):
    return median(s)


@per_sc("min")
def f_min(s, t, c):
    return min(s)


@per_sc("max")
def f_max(s, t, c):
    return max(s)


@per_sc("q10")
def f_q10(s, t, c):
    return quantile(s, 0.10)


@per_sc("q90")
def f_q90(s, t, c):
    return quantile(s, 0.90)


# --- fluctuation
@per_sc("var")
def f_var(s, t, c):
    return variance(s)


@per_sc("std")
def f_std(s, t, c):
    return stdev(s)


@per_sc("cv")
def f_cv(s, t, c):
    """Coefficient of variation: fluctuation NORMALISED by level.

    Matters because a hand attenuates the signal, shrinking absolute variance
    even as relative agitation rises. Raw variance can move the wrong way.
    """
    m = mean(s)
    return stdev(s) / m if m > 0 else 0.0


@per_sc("mad")
def f_mad(s, t, c):
    d = [abs(s[i + 1] - s[i]) for i in range(len(s) - 1)]
    return mean(d)


@per_sc("mad_norm")
def f_mad_norm(s, t, c):
    m = mean(s)
    d = [abs(s[i + 1] - s[i]) for i in range(len(s) - 1)]
    return mean(d) / m if m > 0 else 0.0


@per_sc("range")
def f_range(s, t, c):
    return max(s) - min(s)


@per_sc("iqr")
def f_iqr(s, t, c):
    return quantile(s, 0.75) - quantile(s, 0.25)


# --- distribution
@per_sc("skew")
def f_skew(s, t, c):
    return skewness(s)


@per_sc("kurtosis")
def f_kurt(s, t, c):
    return kurtosis(s)


@per_sc("entropy")
def f_entropy(s, t, c):
    return entropy(s)


# --- temporal / spectral
@per_sc("bp_breath")
def f_bp_breath(s, t, c):
    """0.1-0.5 Hz: breathing. The band that could reveal a STATIONARY person."""
    return band_power(_spec_of(s, t, c), 0.1, 0.5)


@per_sc("bp_slow")
def f_bp_slow(s, t, c):
    """0.01-0.1 Hz: slow body motion, posture shifts, drift."""
    return band_power(_spec_of(s, t, c), 0.01, 0.1)


@per_sc("bp_motion")
def f_bp_motion(s, t, c):
    """0.5-2 Hz: limb motion, walking cadence."""
    return band_power(_spec_of(s, t, c), 0.5, 2.0)


@per_sc("bp_fast")
def f_bp_fast(s, t, c):
    """2-5 Hz: fast motion and noise."""
    return band_power(_spec_of(s, t, c), 2.0, 5.0)


@per_sc("bp_ratio_breath")
def f_bp_ratio(s, t, c):
    """Breathing band relative to total: normalises out overall power changes."""
    spec = _spec_of(s, t, c)
    tot = sum(p for _f, p in spec)
    return band_power(spec, 0.1, 0.5) / tot if tot > 0 else 0.0


@per_sc("spec_centroid")
def f_centroid(s, t, c):
    return spectral_centroid(_spec_of(s, t, c))


@per_sc("spec_flatness")
def f_flatness(s, t, c):
    return spectral_flatness(_spec_of(s, t, c))


@per_sc("acf_lag1")
def f_acf1(s, t, c):
    return autocorr(s, 1)


@per_sc("acf_lag5")
def f_acf5(s, t, c):
    return autocorr(s, 5)


@per_sc("acf_lag20")
def f_acf20(s, t, c):
    """Long-lag correlation: high means slow, structured change."""
    return autocorr(s, 20)


@per_sc("zcr")
def f_zcr(s, t, c):
    return zero_crossings(s)


@per_sc("drift")
def f_drift(s, t, c):
    """Difference between the two halves of the segment: within-segment drift."""
    h = len(s) // 2
    return abs(mean(s[:h]) - mean(s[h:]))


# ------------------------------------------------------ segment-level (scalar)
# Each takes the segment dict (with per-sc series available) and returns a scalar.

SEGMENT = {}


def segment(name):
    def deco(fn):
        SEGMENT[name] = fn
        return fn
    return deco


@segment("band_mean")
def s_band_mean(seg, sc_series, ts, ctx):
    return mean([mean(s) for s in sc_series])


@segment("band_var")
def s_band_var(seg, sc_series, ts, ctx):
    return mean([variance(s) for s in sc_series])


@segment("spectral_tilt")
def s_tilt(seg, sc_series, ts, ctx):
    """Slope of mean amplitude across subcarrier index.

    A body absorbing/reflecting unevenly across the band tilts the spectrum.
    Frequency-selective effects show up here but not in a band average.
    """
    prof = [mean(s) for s in sc_series]
    n = len(prof)
    xm = (n - 1) / 2
    sxx = sum((i - xm) ** 2 for i in range(n))
    return (sum((i - xm) * prof[i] for i in range(n)) / sxx) if sxx else 0.0


@segment("spectral_curvature")
def s_curv(seg, sc_series, ts, ctx):
    """Second difference of the profile: how 'bumpy' the frequency response is.

    Multipath nulls create bumps; a clean line-of-sight channel is smoother.
    """
    prof = [mean(s) for s in sc_series]
    if len(prof) < 3:
        return 0.0
    d2 = [prof[i + 1] - 2 * prof[i] + prof[i - 1] for i in range(1, len(prof) - 1)]
    return mean([abs(v) for v in d2])


@segment("n_nulls")
def s_nulls(seg, sc_series, ts, ctx):
    """Count of subcarriers well below the band mean (deep fades).

    A body creates new nulls by adding reflection paths that cancel.
    """
    prof = [mean(s) for s in sc_series]
    m = mean(prof)
    return sum(1 for p in prof if p < 0.5 * m)


@segment("profile_entropy")
def s_prof_entropy(seg, sc_series, ts, ctx):
    """Entropy of the normalised frequency profile: how evenly power spreads."""
    prof = [mean(s) for s in sc_series]
    tot = sum(prof)
    if tot <= 0:
        return 0.0
    p = [v / tot for v in prof if v > 0]
    return -sum(v * math.log(v) for v in p)


@segment("sc_coherence")
def s_coherence(seg, sc_series, ts, ctx):
    """Mean correlation between ADJACENT subcarriers' time series.

    A single dominant scatterer moves neighbouring subcarriers together, so
    coherence rises; independent noise keeps it low. This is a cross-subcarrier
    structure measure that no per-subcarrier statistic can see.
    """
    cs = []
    for i in range(len(sc_series) - 1):
        cs.append(pearson(sc_series[i], sc_series[i + 1]))
    return mean(cs)


@segment("sc_coherence_far")
def s_coherence_far(seg, sc_series, ts, ctx):
    """Correlation between subcarriers far apart in frequency.

    Distinguishes wideband (whole-channel) effects from narrowband ones.
    """
    n = len(sc_series)
    cs = []
    for i in range(n // 2):
        cs.append(pearson(sc_series[i], sc_series[i + n // 2]))
    return mean(cs)


@segment("eff_rank")
def s_eff_rank(seg, sc_series, ts, ctx):
    """Participation ratio of subcarrier variances — an 'effective dimension'.

    Low when one subcarrier dominates the variance, high when many contribute.
    A cheap stand-in for PCA rank without needing an eigensolver.
    """
    v = [variance(s) for s in sc_series]
    s1 = sum(v)
    s2 = sum(x * x for x in v)
    return (s1 * s1 / s2) if s2 > 0 else 0.0


@segment("dominant_frac")
def s_dominant(seg, sc_series, ts, ctx):
    """Fraction of total variance held by the single most active subcarrier."""
    v = [variance(s) for s in sc_series]
    tot = sum(v)
    return (max(v) / tot) if tot > 0 else 0.0


@segment("rssi_mean")
def s_rssi_mean(seg, sc_series, ts, ctx):
    r = seg.get("rssi") or []
    return mean(r)


@segment("rssi_std")
def s_rssi_std(seg, sc_series, ts, ctx):
    r = seg.get("rssi") or []
    return stdev(r)


@segment("frame_rate")
def s_rate(seg, sc_series, ts, ctx):
    """Frames per second in this segment.

    Included deliberately as a CONFOUND CHECK, not a detector: a body can block
    packets and lower the rate. If this separates the classes as well as the
    channel features do, an apparent 'presence signal' may be a traffic artefact.
    """
    if len(ts) < 2:
        return 0.0
    span = (ts[-1] - ts[0]) / 1e6
    return len(ts) / span if span > 0 else 0.0


@segment("iat_std")
def s_iat_std(seg, sc_series, ts, ctx):
    """Std-dev of inter-arrival times: burstiness of frame arrival."""
    if len(ts) < 3:
        return 0.0
    d = [(ts[i + 1] - ts[i]) / 1000.0 for i in range(len(ts) - 1)]
    return stdev(d)
