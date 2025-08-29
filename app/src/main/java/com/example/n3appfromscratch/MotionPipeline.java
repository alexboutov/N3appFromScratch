package com.example.n3appfromscratch;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.Locale;

/**
 * Encapsulates all sensor fusion, biasing, filtering, ZUPT, "P" computation, and CSV logging.
 * Activity is now a thin wrapper that forwards SensorEvents and renders the UI text.
 */
public class MotionPipeline {

    private static final String TAG = "MotionPipeline";
    private static final float  G   = 9.80665f; // m/s^2

    // CSV
    private CsvLogger csv;
    private boolean isLogging = false;
    private String header;

    // Config (user-tunable)
    private final MotionConfig cfg;

    // Orientation
    private final float[] R = new float[9];     // device->world (row-major)
    private final float[] E = new float[3];     // azimuth, pitch, roll (radians)
    private boolean hasR = false;

    private double yawDeg = Double.NaN;         // [-180..180]
    private double yawUnwrapped = Double.NaN;
    private double lastYawDeg = Double.NaN;
    private double tiltDeg = Double.NaN;

    // Forward vector capture (device +X mapped into world on START)
    private boolean fwdCaptured = false;
    private final float[] FWD_WORLD = new float[]{1f, 0f, 0f};

    // Latest raw sensors (device frame)
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;

    // Timing (ACC based)
    private long lastAccNs = -1L;

    // World-frame accelerations
    private final float[] aW    = new float[3]; // incl. g
    private final float[] aWlin = new float[3]; // minus g

    // Forward accel & velocity
    private double aFwdRaw = Double.NaN;
    private double aFwd = Double.NaN;          // bias-corrected (pre-HPF)
    private double aFwdHPF = Double.NaN;       // for UI velocity integration
    private double lastAFwdRaw = Double.NaN;
    private double lastAFwd = Double.NaN;
    private double lastAFwdHPF = Double.NaN;
    private double vFwdRaw = 0.0;
    private double vFwd = 0.0;

    // Bias calibration (mean over first window, then frozen)
    private boolean calibrating = false;
    private long calibStartNs = -1L;
    private double sumBias = 0.0;
    private long nBias = 0L;
    private double aBias = 0.0;

    // HPF
    private OnePoleHPF hpf;

    // ZUPT (turn-span)
    private static final class YawSample {
        final double t, yawDegUnwrapped, tiltDeg;
        YawSample(double t, double yaw, double tiltDeg) { this.t = t; this.yawDegUnwrapped = yaw; this.tiltDeg = tiltDeg; }
    }
    private final Deque<YawSample> yawBuf = new ArrayDeque<>();
    private static final int TURN_BUF_MAX = 512;
    private double lastZuputTSec = -1.0;
    private int zuptCount = 0;

    // Stats (for footer)
    private long rowsLogged = 0;
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

    // stats for raw/corrected
    private long nARaw = 0, nACorr = 0;
    private double sumARaw = 0.0, sumA2Raw = 0.0, sumAbsARaw = 0.0;
    private double sumACorr = 0.0, sumA2Corr = 0.0, sumAbsACorr = 0.0;
    private final ArrayList<Double> absARawList = new ArrayList<>();
    private final ArrayList<Double> absACorrList = new ArrayList<>();
    private double t0Raw = Double.NaN, sumXRaw = 0.0, sumX2Raw = 0.0, sumYRaw = 0.0, sumXYRaw = 0.0;
    private double t0Corr = Double.NaN, sumXCorr = 0.0, sumX2Corr = 0.0, sumYCorr = 0.0, sumXYCorr = 0.0;

    // --- Legacy "P" logic inputs/state (GRAVITY + MAG + LINEAR_ACCELERATION) ---
    private final float[] latestGravity = new float[3];
    private final float[] latestMag = new float[3];
    private boolean hasGrav = false;
    private boolean hasMag  = false;
    private float latestP = Float.NaN; // updated on LINEAR_ACCELERATION and logged with ACC row

    public MotionPipeline(MotionConfig config) {
        this.cfg = config != null ? config : new MotionConfig();
        this.hpf = new OnePoleHPF(this.cfg.HPF_FC_HZ);
    }

    /** Start logging with a given CsvLogger and CSV header. */
    public void startLogging(CsvLogger csv, String header) {
        this.csv = csv;
        this.header = header;
        if (this.csv != null && this.header != null) {
            this.csv.start(this.header);
            Log.i(TAG, this.header);
        }
        resetForStart();
        isLogging = true;
    }

    /** Stop, return stats footer lines, and detach the logger. */
    public String[] stopAndGetFooter() {
        String[] footer = buildStatsFooter();
        isLogging = false;
        this.csv = null; // Activity will finish CSV with footer
        return footer;
    }

    /** Handle any SensorEvent; logs rows inside ACC branch. */
    public void handleEvent(SensorEvent event) {
        final int type = event.sensor.getType();
        final double tSec = event.timestamp / 1e9;

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR: {
                SensorManager.getRotationMatrixFromVector(R, event.values);
                hasR = true;

                SensorManager.getOrientation(R, E);
                double yawNow = Math.toDegrees(E[0]);
                yawDeg = yawNow;

                if (Double.isNaN(yawUnwrapped)) {
                    yawUnwrapped = yawDeg;
                } else {
                    double dy = yawDeg - lastYawDeg;
                    if (dy > 180) dy -= 360;
                    else if (dy < -180) dy += 360;
                    yawUnwrapped += dy;
                }
                lastYawDeg = yawDeg;

                double cz = clamp(R[8], -1.0, 1.0);
                tiltDeg = Math.toDegrees(Math.acos(cz));
                break;
            }

            case Sensor.TYPE_GRAVITY: {
                latestGravity[0] = event.values[0];
                latestGravity[1] = event.values[1];
                latestGravity[2] = event.values[2];
                hasGrav = true;
                break;
            }

            case Sensor.TYPE_LINEAR_ACCELERATION: {
                if (hasGrav && hasMag) {
                    latestP = NataDataCollect.computeP(latestGravity, latestMag, event.values, cfg.poolAzimuthDeg);
                } else {
                    latestP = Float.NaN;
                }
                break;
            }

            case Sensor.TYPE_GYROSCOPE: {
                gyrX = event.values[0];
                gyrY = event.values[1];
                gyrZ = event.values[2];
                break;
            }

            case Sensor.TYPE_MAGNETIC_FIELD: {
                magX = event.values[0];
                magY = event.values[1];
                magZ = event.values[2];
                latestMag[0] = magX;
                latestMag[1] = magY;
                latestMag[2] = magZ;
                hasMag = true;
                break;
            }

            case Sensor.TYPE_ACCELEROMETER: {
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];

                if (!isLogging) break;

                // dt (ACC clock)
                double dtSec;
                if (lastAccNs > 0) dtSec = (event.timestamp - lastAccNs) / 1_000_000_000.0;
                else dtSec = 0.0;
                if (calibrating && calibStartNs < 0L) calibStartNs = event.timestamp;
                lastAccNs = event.timestamp;

                // one-time forward vector capture when R is available
                if (!fwdCaptured && hasR) {
                    FWD_WORLD[0] = R[0];
                    FWD_WORLD[1] = R[3];
                    FWD_WORLD[2] = R[6];
                    normalize3(FWD_WORLD);
                    Log.i(TAG, String.format(Locale.US, "FWD captured(world): [%.4f, %.4f, %.4f]",
                            FWD_WORLD[0], FWD_WORLD[1], FWD_WORLD[2]));
                    fwdCaptured = true;
                }

                // world linear accel
                if (hasR && fwdCaptured) {
                    mulR(aW, R, accX, accY, accZ);
                    aWlin[0] = aW[0];
                    aWlin[1] = aW[1];
                    aWlin[2] = aW[2] - G;

                    aFwdRaw = aWlin[0] * FWD_WORLD[0] + aWlin[1] * FWD_WORLD[1] + aWlin[2] * FWD_WORLD[2];
                } else {
                    aFwdRaw = Double.NaN;
                }

                // bias calibration
                if (calibrating && !Double.isNaN(aFwdRaw)) {
                    sumBias += aFwdRaw;
                    nBias++;
                    double elapsed = (event.timestamp - calibStartNs) / 1_000_000_000.0;
                    if (elapsed >= cfg.CAL_WINDOW_S) {
                        aBias = (nBias > 0) ? (sumBias / nBias) : 0.0;
                        calibrating = false;
                        Log.i(TAG, String.format(Locale.US, "Bias frozen: aBias=%.6f (n=%d, %.2fs)", aBias, nBias, elapsed));
                    }
                }

                // corrected accel
                double biasNow = (!calibrating) ? aBias : (nBias > 0 ? (sumBias / nBias) : 0.0);
                aFwd = (!Double.isNaN(aFwdRaw)) ? (aFwdRaw - biasNow) : Double.NaN;

                // HPF + integrate
                aFwdHPF = (!Double.isNaN(aFwd)) ? hpf.filter(aFwd, dtSec) : Double.NaN;

                if (!Double.isNaN(aFwdRaw) && !Double.isNaN(lastAFwdRaw) && dtSec > 0)
                    vFwdRaw += 0.5 * (aFwdRaw + lastAFwdRaw) * dtSec;
                if (!Double.isNaN(aFwdHPF) && !Double.isNaN(lastAFwdHPF) && dtSec > 0)
                    vFwd += 0.5 * (aFwdHPF + lastAFwdHPF) * dtSec;

                if (!Double.isNaN(aFwdRaw))  lastAFwdRaw  = aFwdRaw;
                if (!Double.isNaN(aFwd))     lastAFwd     = aFwd;
                if (!Double.isNaN(aFwdHPF))  lastAFwdHPF  = aFwdHPF;

                // ZUPT turn detector
                if (!Double.isNaN(yawUnwrapped)) {
                    yawBuf.addLast(new YawSample(tSec, yawUnwrapped, tiltDeg));
                    if (yawBuf.size() > TURN_BUF_MAX) yawBuf.removeFirst();
                    while (!yawBuf.isEmpty() && (tSec - yawBuf.peekFirst().t) > cfg.TURN_WINDOW_S) {
                        yawBuf.removeFirst();
                    }
                    checkTurnZUPT(tSec);
                }

                // Log row
                double gyroNorm = Math.sqrt((double)gyrX*gyrX + (double)gyrY*gyrY + (double)gyrZ*gyrZ);
                String row = String.format(Locale.US,
                        "%.6f,%.6f," +                  // t, dt
                                "%.3f,%.3f,%.3f," +            // acc
                                "%.5f,%.5f,%.5f," +            // gyro
                                "%.1f,%.1f,%.1f," +            // mag
                                "%.1f,%.3f,%.1f," +            // yaw, gyroNorm, tilt
                                "%.3f,%.3f,%.4f,%.4f,%.4f",    // aFwd_raw, aFwd, vFwd_raw, vFwd, P
                        tSec, dtSec,
                        accX, accY, accZ,
                        gyrX, gyrY, gyrZ,
                        magX, magY, magZ,
                        yawDeg, gyroNorm, tiltDeg,
                        aFwdRaw, aFwd, vFwdRaw, vFwd, latestP);
                if (csv != null) csv.log(row);
                rowsLogged++;

                // Stats
                if (Double.isNaN(firstTSec)) firstTSec = tSec;
                lastTSec = tSec;
                if (dtSec > 0) {
                    sumDt += dtSec; nDt += 1.0;
                    if (dtSec < minDt) minDt = dtSec;
                    if (dtSec > maxDt) maxDt = dtSec;
                }
                if (!Double.isNaN(aFwdRaw)) {
                    nARaw++; sumARaw += aFwdRaw; sumA2Raw += aFwdRaw * aFwdRaw;
                    sumAbsARaw += Math.abs(aFwdRaw);
                    absARawList.add(Math.abs(aFwdRaw));
                    if (Double.isNaN(t0Raw)) t0Raw = tSec;
                    double x = tSec - t0Raw;
                    sumXRaw += x; sumX2Raw += x*x; sumYRaw += aFwdRaw; sumXYRaw += x*aFwdRaw;
                }
                if (!Double.isNaN(aFwd)) {
                    nACorr++; sumACorr += aFwd; sumA2Corr += aFwd*aFwd;
                    sumAbsACorr += Math.abs(aFwd);
                    absACorrList.add(Math.abs(aFwd));
                    if (Double.isNaN(t0Corr)) t0Corr = tSec;
                    double x = tSec - t0Corr;
                    sumXCorr += x; sumX2Corr += x*x; sumYCorr += aFwd; sumXYCorr += x*aFwd;
                }
                break;
            }
        }
    }

    /** Current UI velocity (HPF-integrated). */
    public double getUiVelocity() { return vFwd; }

    /** Update pool azimuth (degrees). */
    public void setPoolAzimuthDeg(float deg) {
        cfg.poolAzimuthDeg = deg;
        Log.i(TAG, "POOL_AZIMUTH_DEG set to " + deg + "°");
    }

    /** Get current pool azimuth (degrees). */
    public float getPoolAzimuthDeg() { return cfg.poolAzimuthDeg; }

    // ---------- internals ----------

    private void resetForStart() {
        // Timing & physics
        lastAccNs = -1L;
        lastAFwdRaw = lastAFwd = lastAFwdHPF = Double.NaN;
        vFwdRaw = vFwd = 0.0;

        // Bias
        calibrating = true;
        calibStartNs = -1L;
        sumBias = 0.0; nBias = 0L; aBias = 0.0;

        // Orientation / forward
        hasR = false; fwdCaptured = false;

        // ZUPT
        yawBuf.clear(); lastZuputTSec = -1.0; zuptCount = 0;

        // Stats
        rowsLogged = 0;
        firstTSec = Double.NaN; lastTSec = Double.NaN;
        sumDt = 0.0; minDt = Double.POSITIVE_INFINITY; maxDt = 0.0; nDt = 0.0;
        nARaw = 0; sumARaw = 0.0; sumA2Raw = 0.0; sumAbsARaw = 0.0; absARawList.clear();
        t0Raw = Double.NaN; sumXRaw = 0.0; sumX2Raw = 0.0; sumYRaw = 0.0; sumXYRaw = 0.0;
        nACorr = 0; sumACorr = 0.0; sumA2Corr = 0.0; sumAbsACorr = 0.0; absACorrList.clear();
        t0Corr = Double.NaN; sumXCorr = 0.0; sumX2Corr = 0.0; sumYCorr = 0.0; sumXYCorr = 0.0;

        // HPF
        hpf = new OnePoleHPF(cfg.HPF_FC_HZ);

        // P dependencies
        latestP = Float.NaN;
        hasGrav = false; hasMag = false;
    }

    private void checkTurnZUPT(double tSec) {
        if (!cfg.ZUPT_ENABLED) return;
        if (yawBuf.size() < 3) return;

        double minYaw = Double.POSITIVE_INFINITY, maxYaw = Double.NEGATIVE_INFINITY;
        for (YawSample s : yawBuf) {
            if (s.yawDegUnwrapped < minYaw) minYaw = s.yawDegUnwrapped;
            if (s.yawDegUnwrapped > maxYaw) maxYaw = s.yawDegUnwrapped;
        }
        double span = maxYaw - minYaw;
        if (span >= cfg.TURN_YAW_DELTA_DEG) {
            if (lastZuputTSec < 0.0 || (tSec - lastZuputTSec) >= cfg.ZUPT_MIN_INTERVAL_S) {
                vFwd = 0.0; vFwdRaw = 0.0;
                lastZuputTSec = tSec; zuptCount++;
                Log.i(TAG, String.format(Locale.US,
                        "ZUPT@turn t=%.3f yaw_span=%.1f° (window=%.2fs) -> v=0",
                        tSec, span, cfg.TURN_WINDOW_S));
            }
        }
    }

    private String[] buildStatsFooter() {
        double duration = (!Double.isNaN(firstTSec) && !Double.isNaN(lastTSec)) ? (lastTSec - firstTSec) : 0.0;
        double avgDt = (nDt > 0) ? (sumDt / nDt) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;

        double meanARaw = (nARaw > 0) ? (sumARaw / nARaw) : Double.NaN;
        double varARaw  = (nARaw > 1) ? Math.max(0.0, (sumA2Raw / nARaw) - (meanARaw * meanARaw)) : Double.NaN;
        double stdARaw  = (nARaw > 1) ? Math.sqrt(varARaw) : Double.NaN;
        double meanAbsARaw = (nARaw > 0) ? (sumAbsARaw / nARaw) : Double.NaN;

        double p95AbsRaw = percentile95(absARawList);

        double slopeRaw = slopeVsTime(nARaw, t0Raw, sumXRaw, sumX2Raw, sumYRaw, sumXYRaw);

        double meanACorr = (nACorr > 0) ? (sumACorr / nACorr) : Double.NaN;
        double varACorr  = (nACorr > 1) ? Math.max(0.0, (sumA2Corr / nACorr) - (meanACorr * meanACorr)) : Double.NaN;
        double stdACorr  = (nACorr > 1) ? Math.sqrt(varACorr) : Double.NaN;
        double meanAbsACorr = (nACorr > 0) ? (sumAbsACorr / nACorr) : Double.NaN;

        double p95AbsCorr = percentile95(absACorrList);

        double slopeCorr = slopeVsTime(nACorr, t0Corr, sumXCorr, sumX2Corr, sumYCorr, sumXYCorr);

        double tau = 1.0 / (2.0 * Math.PI * cfg.HPF_FC_HZ);

        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- STATS BEGIN ----");
        lines.add(String.format(Locale.US, "# rows_logged=%d", rowsLogged));
        lines.add(String.format(Locale.US, "# duration_s=%.6f", duration));
        lines.add(String.format(Locale.US, "# dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                avgDt, (Double.isInfinite(minDt)?0.0:minDt), maxDt, hz));

        lines.add(String.format(Locale.US, "# aBias=%.6f m/s^2 (calib_n=%d, window_s=%.1f, calibrating=%s)",
                aBias, nBias, cfg.CAL_WINDOW_S, Boolean.toString(calibrating)));

        lines.add(String.format(Locale.US, "# HPF_fc_hz=%.3f tau_s=%.3f (UI velocity integrates HPF(aFwd))",
                cfg.HPF_FC_HZ, tau));

        lines.add(String.format(Locale.US, "# ZUPT_turns=%d (window_s=%.2f, min_span_deg=%.0f, min_interval_s=%.1f)",
                zuptCount, cfg.TURN_WINDOW_S, cfg.TURN_YAW_DELTA_DEG, cfg.ZUPT_MIN_INTERVAL_S));

        lines.add(String.format(Locale.US, "# RAW  aFwd_mean=%.6f aFwd_std=%.6f aFwd_mean_abs=%.6f aFwd_p95_abs=%.6f",
                meanARaw, stdARaw, meanAbsARaw, p95AbsRaw));
        lines.add(String.format(Locale.US, "# RAW  aFwd_slope_vs_time=%.8f m/s^3", slopeRaw));
        lines.add(String.format(Locale.US, "# RAW  vFwd_final=%.6f m/s", vFwdRaw));

        lines.add(String.format(Locale.US, "# CORR aFwd_mean=%.6f aFwd_std=%.6f aFwd_mean_abs=%.6f aFwd_p95_abs=%.6f",
                meanACorr, stdACorr, meanAbsACorr, p95AbsCorr));
        lines.add(String.format(Locale.US, "# CORR aFwd_slope_vs_time=%.8f m/s^3", slopeCorr));
        lines.add(String.format(Locale.US, "# CORR vFwd_final=%.6f m/s", vFwd));

        lines.add("# ---- STATS END ----");
        return lines.toArray(new String[0]);
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    private static void mulR(float[] out, float[] R, float x, float y, float z) {
        out[0] = R[0]*x + R[1]*y + R[2]*z;
        out[1] = R[3]*x + R[4]*y + R[5]*z;
        out[2] = R[6]*x + R[7]*y + R[8]*z;
    }

    private static void normalize3(float[] v) {
        double n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n > 0) { v[0]/=n; v[1]/=n; v[2]/=n; }
    }

    private static double percentile95(ArrayList<Double> list) {
        if (list.isEmpty()) return Double.NaN;
        Collections.sort(list);
        int m = list.size();
        int idx = (int) Math.floor(0.95 * (m - 1));
        idx = Math.max(0, Math.min(idx, m - 1));
        return list.get(idx);
    }

    private static double slopeVsTime(long n, double t0, double sumX, double sumX2, double sumY, double sumXY) {
        if (n <= 1) return Double.NaN;
        double N = (double) n;
        double denom = N * sumX2 - sumX * sumX;
        if (Math.abs(denom) <= 1e-12) return Double.NaN;
        return (N * sumXY - sumX * sumY) / denom;
    }
}
