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
 * Changes in this drop-in:
 *  - LA rotation now prefers ROTATION_VECTOR (RV) and falls back to GRAV+MAG.
 *  - LA rows include azErrDeg (desired azimuth - world azimuth from E/N), wrapped to [-180,180].
 *  - LA sign is defined so motion TOWARD azimuth is positive.
 *  - LA integration uses proper trapezoids (acc->vel, then vel->dist).
 */
public class MotionPipeline {

    private static final String TAG = "MotionPipeline";
    private static final float  G   = 9.80665f; // m/s^2

    // Config
    private final MotionConfig cfg;

    // === CSVs (separate files) ===
    private CsvLogger csvAcc = null;
    private CsvLogger csvLa  = null;

    // === Orientation caches ===
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

    // === ACC pipeline (A): device-forward ===
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

    // === LA pipeline (B): azimuth-forward / legacy P ===
    private long   lastLaNs = -1L;
    private float  P = Float.NaN;            // m/s^2 (projection of world LA onto azimuth)
    private double vP = 0.0;                 // m/s
    private double lastVP = Double.NaN;
    private double distance_vP = 0.0;        // m
    private double lastP_LA = Double.NaN;    // previous P for trapezoid

    // LA extras (world components for transparency)
    private float  E_world = Float.NaN, N_world = Float.NaN, Z_world = Float.NaN;

    // LA stats for footer/averages
    private long   rowsLa = 0;
    private double firstLaTSec = Double.NaN, lastLaTSec = Double.NaN;
    private double sumDtLa = 0.0, minDtLa = Double.POSITIVE_INFINITY, maxDtLa = 0.0, nDtLa = 0.0;

    public MotionPipeline(MotionConfig config) {
        this.cfg = (config != null) ? config : new MotionConfig();
    }

    // ------------ lifecycle ------------

    public void start(CsvLogger csvAcc, String headerAcc, CsvLogger csvLa, String headerLa) {
        this.csvAcc = csvAcc;
        this.csvLa  = csvLa;
        resetAll();
        // headers already written in MainActivity
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
        P = Float.NaN; vP = 0.0; lastVP = Double.NaN; distance_vP = 0.0;
        lastP_LA = Double.NaN;
        E_world = N_world = Z_world = Float.NaN;
        rowsLa = 0; firstLaTSec = Double.NaN; lastLaTSec = Double.NaN;
        sumDtLa = 0.0; minDtLa = Double.POSITIVE_INFINITY; maxDtLa = 0.0; nDtLa = 0.0;
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
        lines.add(String.format(Locale.US, "# FINAL distance_vP=%.3f m", distance_vP));
        return lines.toArray(new String[0]);
    }

    // ------------ core event handler ------------

    public void handleEvent(SensorEvent event) {
        final int type = event.sensor.getType();

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR: {
                SensorManager.getRotationMatrixFromVector(R_rv, event.values);
                hasRv = true;
                SensorManager.getOrientation(R_rv, E_rv);
                yawDeg = Math.toDegrees(E_rv[0]);
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

                // World rotation: prefer GM (gravity+mag) for world axes, fallback to RV
                boolean haveGM = hasGrav && hasMag && SensorManager.getRotationMatrix(R_gm, I_gm, latestGravity, latestMag);
                float[] R = haveGM ? R_gm : (hasRv ? R_rv : null);
                if (R == null) {
                    logAccRow(tAcc, dtAcc, Double.isNaN(yawDeg) ? Double.NaN : yawDeg,
                            Double.isNaN(tiltDeg) ? Double.NaN : tiltDeg,
                            Double.NaN);
                    break;
                }

                // One-time capture of device-forward in world
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

                // World linear acceleration from ACC
                mulR(aW, R, accX, accY, accZ);
                aWlin[0] = aW[0];
                aWlin[1] = aW[1];
                aWlin[2] = aW[2] - G;

                // Projection onto device-forward axis
                aFwdRaw = aWlin[0]*FWD_WORLD[0] + aWlin[1]*FWD_WORLD[1] + aWlin[2]*FWD_WORLD[2];

                // Bias (freeze after window)
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

                // Log ACC row
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

                double tLa = event.timestamp / 1e9;
                double dtLa;
                if (lastLaNs > 0) dtLa = (event.timestamp - lastLaNs) / 1_000_000_000.0;
                else dtLa = 0.0;
                lastLaNs = event.timestamp;

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
                    P = (float)(-(E_world * Math.sin(rad) + N_world * Math.cos(rad)));

                    // Instantaneous world azimuth from E/N and its error vs desired
                    double azFromEN = Math.toDegrees(Math.atan2(E_world, N_world));
                    if (azFromEN < 0) azFromEN += 360.0;
                    azErr = cfg.poolAzimuthDeg - azFromEN; // desired - measured
                    if (azErr > 180) azErr -= 360;
                    if (azErr < -180) azErr += 360;
                } else {
                    P = Float.NaN;
                }

                // Integrations (proper trapezoids)
                double prevP = lastP_LA;
                double vPprev = vP;

                if (!Double.isNaN(P) && dtLa > 0) {
                    if (!Double.isNaN(prevP)) vP += 0.5 * (P + prevP) * dtLa; // a -> v
                    else                       vP += P * dtLa;
                }
                if (!Double.isNaN(vP) && !Double.isNaN(vPprev) && dtLa > 0) {
                    distance_vP += 0.5 * (vP + vPprev) * dtLa;               // v -> s
                }

                lastP_LA = P;
                lastVP = vP;

                // Log LA row (now includes azErrDeg at the end)
                logLaRow(tLa, dtLa, yawDeg, tiltDeg, azErr);

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

    private void logLaRow(double tLa, double dtLa, double yawDegNow, double tiltDegNow, double azErrDeg) {
        if (csvLa == null) return;
        String row = String.format(Locale.US,
                "%.6f,%.6f," +            // tLa, dtLa
                        "%.3f,%.3f,%.3f," +      // laX,laY,laZ
                        "%.1f,%.1f,%.1f," +      // yawDeg, tiltDeg, azimuthDeg
                        "%.4f,%.4f,%.4f," +      // P, vP, distance_vP
                        "%.4f,%.4f,%.4f," +      // E_world, N_world, Z_world
                        "%.1f",                   // azErrDeg
                tLa, dtLa,
                laX, laY, laZ,
                yawDegNow, tiltDegNow, (double)cfg.poolAzimuthDeg,
                P, vP, distance_vP,
                E_world, N_world, Z_world,
                azErrDeg
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
}
