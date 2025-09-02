package com.example.n3appfromscratch;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Two independent pipelines + logs:
 *  A) ACC (device-forward) -> dt from ACC, logs to csvAcc
 *  B) LA  (azimuth-forward/"P") -> dt from LA, logs to csvLa
 * No mixing, no alignment, no interpolation.
 *
 * This drop-in adds LA-only:
 *  - Stillness detector (gyro-norm + LA-RMS EMA)
 *  - P-bias freeze/update during stillness
 *  - vP ZUPT while still (exponential bleed toward 0)
 *  - Optional tiny HPF on P_corr before integration
 *  - Diagnostics for timestamp issues: raw ns, dt ns, dtFlag, sensor info & counters
 *
 * NOTE: The "P" column in the LA CSV logs the corrected value (P_corr)
 *       that is actually used for integration.
 */
public class MotionPipeline {

    private static final String TAG = "MotionPipeline";
    private static final float  G   = 9.80665f; // m/s^2

    // === LA stillness / bias / ZUPT tuning (LA pipeline only) ===
    private static final double STILL_GYRO_RAD_S   = 0.30;  // gyro-norm threshold
    private static final double STILL_LA_RMS_MS2   = 0.12;  // linear-acc RMS threshold
    private static final double LA_RMS_TAU_S       = 0.50;  // EMA time constant for LA RMS
    private static final double BIAS_FREEZE_S      = 0.80;  // min stillness to (re)freeze P_bias
    private static final double ZUPT_ENABLE_S      = 0.60;  // stillness time before ZUPT applies
    private static final double ZUPT_TAU_S         = 0.70;  // vP exponential bleed time-constant

    // Optional tiny HPF on P_corr (pre-integration)
    private static final boolean HPF_ENABLED       = true;
    private static final double  HPF_FC_HZ         = 0.08;

    private final MotionConfig cfg;

    // === CSVs (separate files) ===
    private CsvLogger csvAcc = null;
    private CsvLogger csvLa  = null;

    // === Orientation caches (RV-first; GM fallback) ===
    private final float[] R_rv = new float[9];
    private final float[] E_rv = new float[3];
    private boolean hasRv = false;
    private double yawDeg = Double.NaN;
    private double tiltDeg = Double.NaN;

    private final float[] R_gm = new float[9];
    private final float[] I_gm = new float[9];
    private final float[] latestGravity = new float[3];
    private final float[] latestMag     = new float[3];
    private boolean hasGrav = false, hasMag = false;

    // === Latest raw sensors (device frame) ===
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;
    private float laX  = Float.NaN, laY  = Float.NaN, laZ  = Float.NaN;
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;

    // ------------ ACC pipeline (device-forward) ------------
    private long   lastAccNs = -1L;
    private final float[] aW = new float[3];      // world accel incl. g
    private final float[] aWlin = new float[3];   // world minus g
    private final float[] FWD_WORLD = new float[]{1f,0f,0f};
    private boolean fwdCaptured = false;
    private double  fwdAzDeg = Double.NaN;        // FDU, degrees

    private double aFwdRaw = Double.NaN;
    private double aFwd = Double.NaN;
    private double vFwdRaw = 0.0;
    private double vFwd = 0.0;
    private double lastAFwdRaw = Double.NaN;
    private double lastAFwd = Double.NaN;
    private double lastVfwd = Double.NaN;
    private double distance_vFwd = 0.0;

    // Simple bias (freeze after window) for ACC forward
    private boolean calibratingAcc = false;
    private long calibStartAccNs = -1L;
    private double sumBiasAcc = 0.0;
    private long nBiasAcc = 0L;
    private double aBiasAcc = 0.0;

    // ACC stats for footer/averages
    private long   rowsAcc = 0;
    private double firstAccTSec = Double.NaN, lastAccTSec = Double.NaN;
    private double sumDtAcc = 0.0, minDtAcc = Double.POSITIVE_INFINITY, maxDtAcc = 0.0, nDtAcc = 0.0;

    // ------------ LA pipeline (azimuth-forward / legacy P) ------------
    private long   lastLaNs = -1L;

    // New: cumulative LA time (starts at 0 on START)
    private double laElapsedSec = 0.0;

    // Raw & corrected P
    private float  P_raw = Float.NaN;   // projection before corrections
    private float  P_corr = Float.NaN;  // after bias & optional HPF
    private double pBias = 0.0;         // estimated DC bias (m/s^2)

    private double vP = 0.0;
    private double lastVP = Double.NaN;
    private double distance_vP = 0.0;
    private double lastP_LA = Double.NaN;

    // Debug world components
    private float  E_world = Float.NaN, N_world = Float.NaN, Z_world = Float.NaN;

    private long   rowsLa = 0;
    private double firstLaTSec = Double.NaN, lastLaTSec = Double.NaN;
    private double sumDtLa = 0.0, minDtLa = Double.POSITIVE_INFINITY, maxDtLa = 0.0, nDtLa = 0.0;

    // Stillness / bias / ZUPT state
    private double laSqEma = Double.NaN;
    private double stillTimer = 0.0;
    private boolean isStill = false;
    private double pBiasSum = 0.0;
    private long   pBiasN = 0L;

    // Optional HPF state on P_corr
    private final OnePoleHPF hpfP = new OnePoleHPF(HPF_FC_HZ);

    // ---- diagnostics for dt issues & sensor identity ----
    private long dtLaNegCount = 0L, dtLaZeroCount = 0L;
    private long minDtLaNs = Long.MAX_VALUE, maxNegDtLaNs = 0L;
    private boolean laSensorInfoLogged = false;
    private String laSensorName = null, laSensorVendor = null;
    private int laSensorVersion = 0;

    public MotionPipeline(MotionConfig config) {
        this.cfg = (config != null) ? config : new MotionConfig();
    }

    // ------------ lifecycle ------------

    public void start(CsvLogger csvAcc, String headerAcc, CsvLogger csvLa, String headerLa) {
        this.csvAcc = csvAcc;
        this.csvLa  = csvLa;
        resetAll();
        // MainActivity writes headers; nothing to do here.
    }

    public void stop() {
        resetAll();
        this.csvAcc = null;
        this.csvLa  = null;
    }

    private void resetAll() {
        // Orientation
        hasRv = false; yawDeg = Double.NaN; tiltDeg = Double.NaN;
        hasGrav = hasMag = false;

        // ACC pipeline
        lastAccNs = -1L;
        fwdCaptured = false;
        fwdAzDeg = Double.NaN;
        aFwdRaw = aFwd = Double.NaN;
        vFwdRaw = vFwd = 0.0;
        lastAFwdRaw = lastAFwd = lastVfwd = Double.NaN;
        distance_vFwd = 0.0;
        calibratingAcc = true; calibStartAccNs = -1L;
        sumBiasAcc = 0.0; nBiasAcc = 0L; aBiasAcc = 0.0;
        rowsAcc = 0; firstAccTSec = Double.NaN; lastAccTSec = Double.NaN;
        sumDtAcc = 0.0; minDtAcc = Double.POSITIVE_INFINITY; maxDtAcc = 0.0; nDtAcc = 0.0;

        // LA pipeline
        lastLaNs = -1L;
        laElapsedSec = 0.0;
        P_raw = Float.NaN; P_corr = Float.NaN; pBias = 0.0;
        vP = 0.0; lastVP = Double.NaN; distance_vP = 0.0; lastP_LA = Double.NaN;
        E_world = N_world = Z_world = Float.NaN;
        rowsLa = 0; firstLaTSec = Double.NaN; lastLaTSec = Double.NaN;
        sumDtLa = 0.0; minDtLa = Double.POSITIVE_INFINITY; maxDtLa = 0.0; nDtLa = 0.0;

        laSqEma = Double.NaN; stillTimer = 0.0; isStill = false;
        pBiasSum = 0.0; pBiasN = 0L;
        hpfP.reset();

        dtLaNegCount = 0L; dtLaZeroCount = 0L; minDtLaNs = Long.MAX_VALUE; maxNegDtLaNs = 0L;
        // keep laSensorInfoLogged as-is; it will log once per app run
    }

    // ------------ azimuth API (LA pipeline only) ------------

    /** Set lane azimuth in degrees (0=North, 90=East, 180=South, 270=West). */
    public void setPoolAzimuthDeg(float deg) {
        float norm = (float)(((double)deg % 360.0 + 360.0) % 360.0);
        cfg.poolAzimuthDeg = norm;
        Log.i(TAG, "POOL_AZIMUTH_DEG set to " + cfg.poolAzimuthDeg + "° (LA pipeline)");
    }
    public float getPoolAzimuthDeg() { return cfg.poolAzimuthDeg; }

    // ------------ getters for UI ------------

    /** Instantaneous velocity (ACC pipeline), m/s */
    public double getUiVelocityAcc() { return vFwd; }

    /** Instantaneous velocity (LA pipeline), m/s */
    public double getUiVelocityLa() { return vP; }

    /** Average velocity over the completed run (ACC): distance / duration, m/s */
    public double getAccAvgVel() {
        double dur = (!Double.isNaN(firstAccTSec) && !Double.isNaN(lastAccTSec))
                ? (lastAccTSec - firstAccTSec) : 0.0;
        if (dur <= 0.0) return 0.0;
        return distance_vFwd / dur;
    }

    /** Average velocity over the completed run (LA): distance / duration, m/s */
    public double getLaAvgVel() {
        double dur = (!Double.isNaN(firstLaTSec) && !Double.isNaN(lastLaTSec))
                ? (lastLaTSec - firstLaTSec) : 0.0;
        if (dur <= 0.0) return 0.0;
        return distance_vP / dur;
    }

    // ------------ footers ------------

    public String[] buildAccFooter() {
        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- ACC PIPELINE STATS ----");
        lines.add(String.format(Locale.US, "# rows=%d", rowsAcc));
        double duration = (!Double.isNaN(firstAccTSec) && !Double.isNaN(lastAccTSec)) ? (lastAccTSec - firstAccTSec) : 0.0;
        double avgDt = (nDtAcc > 0) ? (sumDtAcc / nDtAcc) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;
        lines.add(String.format(Locale.US, "# duration_s=%.6f  dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                duration, avgDt, (Double.isInfinite(minDtAcc)?0.0:minDtAcc), maxDtAcc, hz));
        lines.add(String.format(Locale.US, "# aBiasAcc=%.6f", aBiasAcc));
        lines.add(String.format(Locale.US, "# FDU_degrees=%.1f", fwdAzDeg));
        lines.add(String.format(Locale.US, "# FINAL distance_vFwd=%.3f m", distance_vFwd));
        return lines.toArray(new String[0]);
    }

    public String[] buildLaFooter() {
        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- LA PIPELINE STATS ----");
        lines.add(String.format(Locale.US, "# rows=%d", rowsLa));
        double duration = (!Double.isNaN(firstLaTSec) && !Double.isNaN(lastLaTSec)) ? (lastLaTSec - firstLaTSec) : 0.0;
        double avgDt = (nDtLa > 0) ? (sumDtLa / nDtLa) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;
        lines.add(String.format(Locale.US, "# duration_s=%.6f  dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                duration, avgDt, (Double.isInfinite(minDtLa)?0.0:minDtLa), maxDtLa, hz));
        lines.add(String.format(Locale.US, "# azimuthDeg=%.1f", (double)cfg.poolAzimuthDeg));
        lines.add(String.format(Locale.US, "# P_bias=%.5f", pBias));
        lines.add(String.format(Locale.US, "# FINAL distance_vP=%.3f m", distance_vP));
        lines.add(String.format(Locale.US, "# LA sensor: %s | vendor=%s | v=%d",
                String.valueOf(laSensorName), String.valueOf(laSensorVendor), laSensorVersion));
        lines.add(String.format(Locale.US, "# dtLa_neg=%d  dtLa_zero=%d  worst_neg_dt_ns=%d  min_pos_dt_ns=%d",
                dtLaNegCount, dtLaZeroCount, maxNegDtLaNs, (minDtLaNs==Long.MAX_VALUE?0:minDtLaNs)));
        return lines.toArray(new String[0]);
    }

    // ------------ core event handler ------------

    public void handleEvent(SensorEvent event) {
        final int type = event.sensor.getType();

        // Log LA sensor identity once (helps detect multiple virtual LA sensors)
        if (!laSensorInfoLogged && type == Sensor.TYPE_LINEAR_ACCELERATION) {
            laSensorName = event.sensor.getName();
            laSensorVendor = event.sensor.getVendor();
            laSensorVersion = event.sensor.getVersion();
            laSensorInfoLogged = true;
            Log.i(TAG, "LA sensor: " + laSensorName + " | vendor=" + laSensorVendor + " | v=" + laSensorVersion);
        }

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR: {
                SensorManager.getRotationMatrixFromVector(R_rv, event.values);
                hasRv = true;
                SensorManager.getOrientation(R_rv, E_rv);
                yawDeg = Math.toDegrees(E_rv[0]); // azimuth
                double cz = clamp(R_rv[8], -1.0, 1.0);
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

            case Sensor.TYPE_MAGNETIC_FIELD: {
                latestMag[0] = event.values[0];
                latestMag[1] = event.values[1];
                latestMag[2] = event.values[2];
                hasMag = true;
                break;
            }

            case Sensor.TYPE_GYROSCOPE: {
                gyrX = event.values[0];
                gyrY = event.values[1];
                gyrZ = event.values[2];
                break;
            }

            case Sensor.TYPE_ACCELEROMETER: {
                // --------- ACC PIPELINE (A) ---------
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];

                double tAcc = event.timestamp / 1e9;
                double dtAcc;
                if (lastAccNs > 0) dtAcc = (event.timestamp - lastAccNs) / 1_000_000_000.0;
                else dtAcc = 0.0;
                if (calibratingAcc && calibStartAccNs < 0L) calibStartAccNs = event.timestamp;
                lastAccNs = event.timestamp;

                boolean haveGM = hasGrav && hasMag && SensorManager.getRotationMatrix(R_gm, I_gm, latestGravity, latestMag);
                float[] R = haveGM ? R_gm : (hasRv ? R_rv : null);
                if (R == null) {
                    logAccRow(tAcc, dtAcc, yawDeg, tiltDeg, Double.NaN);
                    break;
                }

                if (!fwdCaptured) {
                    FWD_WORLD[0] = R[0]; // world E component of device +X
                    FWD_WORLD[1] = R[3]; // world N component of device +X
                    FWD_WORLD[2] = R[6]; // world Z component of device +X
                    normalize3(FWD_WORLD);
                    fwdAzDeg = Math.toDegrees(Math.atan2(FWD_WORLD[0], FWD_WORLD[1])); // atan2(E, N)
                    if (fwdAzDeg < 0) fwdAzDeg += 360.0;
                    Log.i(TAG, String.format(Locale.US,
                            "FWD captured(world): [%.4f, %.4f, %.4f]  az=%.1f°",
                            FWD_WORLD[0], FWD_WORLD[1], FWD_WORLD[2], fwdAzDeg));
                    fwdCaptured = true;
                }

                mulR(aW, R, accX, accY, accZ);          // world accel incl. g
                aWlin[0] = aW[0];
                aWlin[1] = aW[1];
                aWlin[2] = aW[2] - G;                   // remove gravity

                aFwdRaw = aWlin[0]*FWD_WORLD[0] + aWlin[1]*FWD_WORLD[1] + aWlin[2]*FWD_WORLD[2];

                if (calibratingAcc && !Double.isNaN(aFwdRaw)) {
                    sumBiasAcc += aFwdRaw; nBiasAcc++;
                    double elapsed = (event.timestamp - calibStartAccNs) / 1e9;
                    if (elapsed >= cfg.CAL_WINDOW_S) {
                        aBiasAcc = (nBiasAcc > 0) ? (sumBiasAcc / nBiasAcc) : 0.0;
                        calibratingAcc = false;
                        Log.i(TAG, String.format(Locale.US,
                                "ACC bias frozen: aBiasAcc=%.6f (n=%d, %.2fs)",
                                aBiasAcc, nBiasAcc, elapsed));
                    }
                }
                double biasNow = (!calibratingAcc) ? aBiasAcc : (nBiasAcc > 0 ? (sumBiasAcc / nBiasAcc) : 0.0);
                aFwd = (!Double.isNaN(aFwdRaw)) ? (aFwdRaw - biasNow) : Double.NaN;

                // Integrations (trapezoid)
                if (!Double.isNaN(aFwdRaw) && !Double.isNaN(lastAFwdRaw) && dtAcc > 0)
                    vFwdRaw += 0.5 * (aFwdRaw + lastAFwdRaw) * dtAcc;
                if (!Double.isNaN(aFwd) && !Double.isNaN(lastAFwd) && dtAcc > 0)
                    vFwd += 0.5 * (aFwd + lastAFwd) * dtAcc;
                if (!Double.isNaN(vFwd) && !Double.isNaN(lastVfwd) && dtAcc > 0)
                    distance_vFwd += 0.5 * (vFwd + lastVfwd) * dtAcc;

                if (!Double.isNaN(aFwdRaw)) lastAFwdRaw = aFwdRaw;
                if (!Double.isNaN(aFwd))    lastAFwd    = aFwd;
                if (!Double.isNaN(vFwd))    lastVfwd    = vFwd;

                logAccRow(tAcc, dtAcc, yawDeg, tiltDeg, fwdAzDeg);

                // dt stats
                if (Double.isNaN(firstAccTSec)) firstAccTSec = tAcc;
                lastAccTSec = tAcc;
                if (dtAcc > 0) {
                    sumDtAcc += dtAcc; nDtAcc += 1.0;
                    if (dtAcc < minDtAcc) minDtAcc = dtAcc;
                    if (dtAcc > maxDtAcc) maxDtAcc = dtAcc;
                }
                break;
            }

            case Sensor.TYPE_LINEAR_ACCELERATION: {
                // --------- LA PIPELINE (B) ---------
                laX = event.values[0];
                laY = event.values[1];
                laZ = event.values[2];

                // Raw ns and dt classification
                long dtNs = (lastLaNs > 0) ? (event.timestamp - lastLaNs) : 0L;
                double dtLa = (lastLaNs > 0) ? (dtNs / 1_000_000_000.0) : 0.0;
                lastLaNs = event.timestamp;

                String dtFlag = "OK";
                if (dtNs == 0L) { dtFlag = "Z"; dtLaZeroCount++; }
                else if (dtNs < 0L) { dtFlag = "NEG"; dtLaNegCount++; if (-dtNs > maxNegDtLaNs) maxNegDtLaNs = -dtNs; }
                if (dtNs > 0 && dtNs < minDtLaNs) minDtLaNs = dtNs;

                // Cumulative elapsed time (only advance on positive dt)
                if (dtNs > 0L) laElapsedSec += dtLa;

                double tLa = event.timestamp / 1e9;

                // Choose rotation source for LA: prefer RV, fallback to GRAV+MAG
                float[] Rused = null;
                if (hasRv) {
                    Rused = R_rv;
                } else if (hasGrav && hasMag && SensorManager.getRotationMatrix(R_gm, I_gm, latestGravity, latestMag)) {
                    Rused = R_gm;
                }

                double azErr = Double.NaN;
                E_world = N_world = Z_world = Float.NaN;
                if (Rused != null) {
                    float[] laW = new float[3];
                    mulR(laW, Rused, laX, laY, laZ);
                    E_world = laW[0]; // +East
                    N_world = laW[1]; // +North
                    Z_world = laW[2]; // +Up

                    // Projection onto desired azimuth (positive toward azimuth)
                    double rad = Math.toRadians(cfg.poolAzimuthDeg);
                    P_raw = (float)(-(E_world * Math.sin(rad) + N_world * Math.cos(rad)));

                    // Instantaneous world azimuth from E/N and its error vs desired
                    double azFromEN = Math.toDegrees(Math.atan2(E_world, N_world));
                    if (azFromEN < 0) azFromEN += 360.0;
                    azErr = cfg.poolAzimuthDeg - azFromEN; // desired - measured
                    if (azErr > 180) azErr -= 360;
                    if (azErr < -180) azErr += 360;
                } else {
                    P_raw = Float.NaN;
                }

                // ---- LA STILLNESS DETECTOR ----
                // gyro norm
                double gyroNorm = Math.sqrt(
                        (double)gyrX*gyrX + (double)gyrY*gyrY + (double)gyrZ*gyrZ);

                // LA RMS via EMA(|LA|^2)
                double laMag = Math.sqrt((double)laX*laX + (double)laY*laY + (double)laZ*laZ);
                if (Double.isNaN(laSqEma) || dtLa <= 0) laSqEma = laMag*laMag;
                else {
                    double alpha = Math.exp(-dtLa / LA_RMS_TAU_S); // 0..1
                    laSqEma = alpha * laSqEma + (1.0 - alpha) * (laMag * laMag);
                }
                double laRms = Math.sqrt(laSqEma);

                boolean prevStill = isStill;
                isStill = (gyroNorm < STILL_GYRO_RAD_S && laRms < STILL_LA_RMS_MS2);

                if (isStill && dtLa > 0) {
                    stillTimer += dtLa;
                    // Accumulate P_raw during stillness to estimate bias
                    if (!Float.isNaN(P_raw)) {
                        pBiasSum += P_raw;
                        pBiasN++;
                    }
                    // Freeze/update bias after sustained stillness
                    if (stillTimer >= BIAS_FREEZE_S && pBiasN >= 5) {
                        pBias = pBiasSum / Math.max(1L, pBiasN);
                    }
                } else {
                    // Reset stillness accumulators when moving
                    stillTimer = 0.0;
                    pBiasSum = 0.0;
                    pBiasN   = 0L;
                }

                // ---- P CORRECTION (bias + optional HPF) ----
                if (!Float.isNaN(P_raw)) {
                    double pTmp = (double)P_raw - pBias;
                    if (HPF_ENABLED && dtLa > 0) {
                        pTmp = hpfP.filter(pTmp, dtLa); // tiny DC shave
                    }
                    P_corr = (float)pTmp;
                } else {
                    P_corr = Float.NaN;
                }

                // ---- INTEGRATIONS (use P_corr) ----
                double prevP = lastP_LA;
                double vPprev = vP;

                if (!Double.isNaN(P_corr) && dtLa > 0) {
                    if (!Double.isNaN(prevP)) vP += 0.5 * (P_corr + prevP) * dtLa; // a -> v
                    else                       vP += P_corr * dtLa;
                }

                // ZUPT during stillness (bleed vP toward 0 when confirmed still)
                if (isStill && stillTimer >= ZUPT_ENABLE_S && dtLa > 0) {
                    double a = Math.exp(-dtLa / ZUPT_TAU_S);
                    vP = a * vP;
                }

                if (!Double.isNaN(vP) && !Double.isNaN(vPprev) && dtLa > 0) {
                    distance_vP += 0.5 * (vP + vPprev) * dtLa;               // v -> s
                }

                lastP_LA = P_corr;
                lastVP = vP;

                // Log LA row (P column logs P_corr)
                logLaRow(tLa, dtLa, laElapsedSec, yawDeg, tiltDeg, azErr,
                        gyroNorm, laRms, isStill,
                        event.timestamp, dtNs, dtFlag);

                // dt stats
                if (Double.isNaN(firstLaTSec)) firstLaTSec = tLa;
                lastLaTSec = tLa;
                if (dtLa > 0) {
                    sumDtLa += dtLa; nDtLa += 1.0;
                    if (dtLa < minDtLa) minDtLa = dtLa;
                    if (dtLa > maxDtLa) maxDtLa = dtLa;
                }
                break;
            }
        }
    }

    // ------------ logging helpers ------------

    private void logAccRow(double tAcc, double dtAcc, double yawDegNow, double tiltDegNow, double fduDeg) {
        if (csvAcc == null) return;
        String row = String.format(Locale.US,
                "%.6f,%.6f," +            // tAcc, dtAcc
                        "%.3f,%.3f,%.3f," +      // accX,accY,accZ
                        "%.1f,%.1f," +           // yawDeg, tiltDeg
                        "%.3f,%.3f,%.4f,%.4f,%.4f," + // aFwd_raw,aFwd,vFwd_raw,vFwd,distance_vFwd
                        "%.1f",                   // FDU_degrees
                tAcc, dtAcc,
                accX, accY, accZ,
                yawDegNow, tiltDegNow,
                aFwdRaw, aFwd, vFwdRaw, vFwd, distance_vFwd,
                fduDeg
        );
        csvAcc.log(row);
        rowsAcc++;
    }

    private void logLaRow(double tLa, double dtLa, double laElapsed,
                          double yawDegNow, double tiltDegNow,
                          double azErrDeg, double gyroNorm, double laRms, boolean isStillNow,
                          long laRawNs, long dtLaNs, String dtFlag) {
        if (csvLa == null) return;
        String row = String.format(Locale.US,
                "%.6f,%.6f,%.6f," +             // tLa, dtLa, LA time elapsed, sec
                        "%.3f,%.3f,%.3f," +            // laX,laY,laZ
                        "%.1f,%.1f,%.1f," +            // yawDeg, tiltDeg, azimuthDeg
                        "%.4f,%.5f,%.4f," +            // P_raw, P_bias, P(=P_corr)
                        "%.4f,%.4f," +                 // vP, distance_vP
                        "%.4f,%.4f,%.4f," +            // E_world, N_world, Z_world
                        "%.1f,%.3f,%.3f,%d," +         // azErrDeg, gyroNorm, laRms, isStill(0/1)
                        "%d,%d,%s",                    // laRawNs, dtLa_ns, dtFlag
                tLa, dtLa, laElapsed,
                laX, laY, laZ,
                yawDegNow, tiltDegNow, (double)cfg.poolAzimuthDeg,
                (double)P_raw, pBias, (double)P_corr,
                vP, distance_vP,
                E_world, N_world, Z_world,
                azErrDeg, gyroNorm, laRms, (isStillNow ? 1 : 0),
                laRawNs, dtLaNs, dtFlag
        );
        csvLa.log(row);
        rowsLa++;
    }

    // ------------ math helpers ------------

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    // out = R * [x,y,z], with R 3x3 row-major
    private static void mulR(float[] out, float[] R, float x, float y, float z) {
        out[0] = R[0]*x + R[1]*y + R[2]*z;
        out[1] = R[3]*x + R[4]*y + R[5]*z;
        out[2] = R[6]*x + R[7]*y + R[8]*z;
    }

    private static void normalize3(float[] v) {
        double n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n > 0) { v[0]/=n; v[1]/=n; v[2]/=n; }
    }

    // Simple one-pole high-pass (stateful, per-sample dt)
    private static final class OnePoleHPF {
        private final double fc;
        private boolean hasState = false;
        private double lastX = 0.0, lastY = 0.0;

        OnePoleHPF(double fcHz) { this.fc = Math.max(0.0, fcHz); }

        void reset() { hasState = false; lastX = 0.0; lastY = 0.0; }

        double filter(double x, double dt) {
            if (fc <= 0.0 || dt <= 0.0) return x; // bypass
            double tau = 1.0 / (2.0 * Math.PI * fc);
            double a = tau / (tau + dt);            // 0..1
            double y;
            if (hasState) {
                y = a * (lastY + x - lastX);
            } else {
                y = 0.0; // start with DC removed (y=0)
                hasState = true;
            }
            lastX = x;
            lastY = y;
            return y;
        }
    }
}
