package com.example.n3appfromscratch;

import androidx.appcompat.app.AppCompatActivity;

import android.os.Bundle;
import android.util.Log;
import android.view.WindowManager;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.content.Context;

import com.example.n3appfromscratch.databinding.ActivityMainBinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";
    private static final String HEADER =
            // Added yawDeg, gyroNorm, tiltDeg; live velocity stays raw
            "t,dt,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,yawDeg,gyroNorm,tiltDeg,aFwd_raw,aFwd,vFwd_raw,vFwd";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accRaw, gyro, mag, rotVec;

    // Logging control
    private boolean isLogging = false;

    // File logger
    private CsvLogger csv;

    // Latest device-frame caches
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;   // ACC raw (m/s^2)
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;   // GYR (rad/s)
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;   // MAG (µT)

    // Orientation
    private final float[] R = new float[9];      // device -> world rotation (row-major)
    private final float[] ORI = new float[3];    // azimuth(yaw), pitch, roll (rad)
    private volatile boolean hasR = false;

    // Capture-forward-at-START (world-frame forward vector equals device +X at START)
    private boolean fwdCaptured = false;
    private final float[] FWD_WORLD = new float[]{1f, 0f, 0f}; // filled at START from R*[1,0,0]

    // ACC timing
    private long lastAccNs = -1L;

    // Physics
    private static final float G = 9.80665f; // m/s^2
    private final float[] aW = new float[3];    // world accel incl. g
    private final float[] aWlin = new float[3]; // world accel minus g
    private double aFwdRaw = Double.NaN;        // no bias removal
    private double aFwd = Double.NaN;           // bias-corrected
    private double lastAFwdRaw = Double.NaN;
    private double lastAFwd = Double.NaN;       // for trapezoid integration
    private double vFwdRaw = 0.0;
    private double vFwd = 0.0;

    // Bias calibration (mean of aFwdRaw over first CAL_WINDOW_S, then frozen)
    private static final double CAL_WINDOW_S = 3.0;
    private boolean calibrating = false;
    private long calibStartNs = -1L;
    private double sumBias = 0.0;
    private long nBias = 0L;
    private double aBias = 0.0; // final frozen bias

    // Stats accumulators (raw and corrected; no filtering)
    private long rowsLogged = 0;           // total CSV rows written
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

    // For raw stats
    private long nARaw = 0;                   // count of valid aFwdRaw
    private double sumARaw = 0.0, sumA2Raw = 0.0, sumAbsARaw = 0.0;
    private final ArrayList<Double> absARawList = new ArrayList<>();
    private double t0Raw = Double.NaN, sumXRaw = 0.0, sumX2Raw = 0.0, sumYRaw = 0.0, sumXYRaw = 0.0;

    // For corrected stats
    private long nACorr = 0;                   // count of valid aFwd
    private double sumACorr = 0.0, sumA2Corr = 0.0, sumAbsACorr = 0.0;
    private final ArrayList<Double> absACorrList = new ArrayList<>();
    private double t0Corr = Double.NaN, sumXCorr = 0.0, sumX2Corr = 0.0, sumYCorr = 0.0, sumXYCorr = 0.0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        accRaw = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro   = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mag    = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        rotVec = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);

        String info = "Sensors: "
                + ((accRaw!=null)?"ACC ":"")
                + ((gyro  !=null)?"GYRO ":"")
                + ((mag   !=null)?"MAG ":"")
                + ((rotVec!=null)?"RV ":"");
        binding.tvInfo.setText(info);

        // START/STOP
        binding.btnStartStop.setOnClickListener(v -> {
            if (!isLogging) {
                // START
                isLogging = true;

                // reset timing & physics
                lastAccNs = -1L;
                lastAFwdRaw = Double.NaN;
                lastAFwd = Double.NaN;
                vFwdRaw = 0.0;
                vFwd = 0.0;

                // bias calib
                calibrating = true;
                calibStartNs = -1L; // will set on first ACC timestamp
                sumBias = 0.0;
                nBias = 0L;
                aBias = 0.0;

                // stats reset
                rowsLogged = 0;
                firstTSec = Double.NaN; lastTSec = Double.NaN;
                sumDt = 0.0; minDt = Double.POSITIVE_INFINITY; maxDt = 0.0; nDt = 0.0;

                nARaw = 0; sumARaw = 0.0; sumA2Raw = 0.0; sumAbsARaw = 0.0; absARawList.clear();
                t0Raw = Double.NaN; sumXRaw = 0.0; sumX2Raw = 0.0; sumYRaw = 0.0; sumXYRaw = 0.0;

                nACorr = 0; sumACorr = 0.0; sumA2Corr = 0.0; sumAbsACorr = 0.0; absACorrList.clear();
                t0Corr = Double.NaN; sumXCorr = 0.0; sumX2Corr = 0.0; sumYCorr = 0.0; sumXYCorr = 0.0;

                // capture forward world vector at START (device +X mapped into world)
                fwdCaptured = false; // will be captured on first ACC when hasR==true

                long avail = CsvLogger.availableBytes(this);
                long total = CsvLogger.totalBytes(this);
                Log.i(TAG, "Storage available: " + CsvLogger.human(avail) + " / " + CsvLogger.human(total));

                long limitBytes = 10L * 1024 * 1024; // 10 MB per part
                Log.i(TAG, "Using per-file limit: " + CsvLogger.human(limitBytes));

                csv = new CsvLogger(this);
                csv.setMaxBytes(limitBytes);
                csv.start(HEADER);
                Log.i(TAG, HEADER);
                binding.btnStartStop.setText("STOP");
                Log.i(TAG, "# START" + (csv != null && csv.getFile()!=null
                        ? " file=" + csv.getFile().getAbsolutePath() : ""));
            } else {
                // STOP
                String[] footer = buildStatsFooter();
                if (csv != null) {
                    // repeat header, then stats lines
                    csv.finishWithFooter(HEADER, footer);
                }
                Log.i(TAG, HEADER);
                for (String s : footer) Log.i(TAG, s);
                isLogging = false;
                binding.btnStartStop.setText("START");
                Log.i(TAG, "# STOP" + (csv != null && csv.getFile()!=null
                        ? " file=" + csv.getFile().getAbsolutePath() : ""));
                csv = null;
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        int rate = SensorManager.SENSOR_DELAY_FASTEST; // maximize rate
        if (rotVec != null) sm.registerListener(this, rotVec, rate);
        if (accRaw != null) sm.registerListener(this, accRaw, rate);
        if (gyro   != null) sm.registerListener(this, gyro,   rate);
        if (mag    != null) sm.registerListener(this, mag,    rate);
        Log.i(TAG, "onResume (listeners registered, FASTEST)");
    }

    @Override
    protected void onPause() {
        super.onPause();
        sm.unregisterListener(this);
        if (csv != null) {
            // If paused while logging, also finalize with footer so file isn't left open.
            String[] footer = buildStatsFooter();
            csv.finishWithFooter(HEADER, footer);
            csv = null;
        }
        if (isLogging) { isLogging = false; binding.btnStartStop.setText("START"); }
        Log.i(TAG, "onPause -> unregister");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final int type = event.sensor.getType();
        final double tSec = event.timestamp / 1e9;

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR:
                SensorManager.getRotationMatrixFromVector(R, event.values);
                hasR = true;
                break;

            case Sensor.TYPE_ACCELEROMETER:
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];

                if (!isLogging) break;

                // dt from ACC timestamps
                double dtSec;
                if (lastAccNs > 0) {
                    dtSec = (event.timestamp - lastAccNs) / 1_000_000_000.0;
                } else {
                    dtSec = 0.0;
                }
                if (calibrating && calibStartNs < 0L) calibStartNs = event.timestamp;
                lastAccNs = event.timestamp;

                // capture forward world vector at START (once, when R available)
                if (!fwdCaptured && hasR) {
                    // world forward = R * [1,0,0] (device +X axis in world coords)
                    FWD_WORLD[0] = R[0]*1f + R[1]*0f + R[2]*0f;
                    FWD_WORLD[1] = R[3]*1f + R[4]*0f + R[5]*0f;
                    FWD_WORLD[2] = R[6]*1f + R[7]*0f + R[8]*0f;
                    normalize3(FWD_WORLD);
                    fwdCaptured = true;
                    Log.i(TAG, String.format(Locale.US,
                            "FWD captured (world): [%.4f, %.4f, %.4f]", FWD_WORLD[0], FWD_WORLD[1], FWD_WORLD[2]));
                }

                // compute world linear accel = R*acc - g*Z_world
                if (hasR && fwdCaptured) {
                    mulR(aW, R, accX, accY, accZ);   // world accel incl. g
                    aWlin[0] = aW[0];
                    aWlin[1] = aW[1];
                    aWlin[2] = aW[2] - G;            // remove gravity (world Z up)

                    // raw forward accel (no bias removal)
                    aFwdRaw = aWlin[0]*FWD_WORLD[0] + aWlin[1]*FWD_WORLD[1] + aWlin[2]*FWD_WORLD[2];
                } else {
                    aFwdRaw = Double.NaN;
                }

                // bias estimate during the first CAL_WINDOW_S (mean of aFwdRaw)
                if (calibrating && !Double.isNaN(aFwdRaw)) {
                    sumBias += aFwdRaw;
                    nBias++;
                    double elapsed = (event.timestamp - calibStartNs) / 1_000_000_000.0;
                    if (elapsed >= CAL_WINDOW_S) {
                        aBias = (nBias > 0) ? (sumBias / nBias) : 0.0;
                        calibrating = false;
                        Log.i(TAG, String.format(Locale.US,
                                "Bias frozen: aBias=%.6f m/s^2 (n=%d, %.2fs)",
                                aBias, nBias, elapsed));
                    }
                }

                // corrected accel = raw - (frozen bias if available, else running mean)
                double biasNow;
                if (!calibrating) {
                    biasNow = aBias;
                } else {
                    biasNow = (nBias > 0) ? (sumBias / nBias) : 0.0;
                }
                aFwd = (!Double.isNaN(aFwdRaw)) ? (aFwdRaw - biasNow) : Double.NaN;

                // integrate both (trapezoid) when we have prior samples and dt>0
                if (!Double.isNaN(aFwdRaw) && !Double.isNaN(lastAFwdRaw) && dtSec > 0) {
                    vFwdRaw += 0.5 * (aFwdRaw + lastAFwdRaw) * dtSec;
                }
                if (!Double.isNaN(aFwd) && !Double.isNaN(lastAFwd) && dtSec > 0) {
                    vFwd += 0.5 * (aFwd + lastAFwd) * dtSec;
                }
                if (!Double.isNaN(aFwdRaw)) lastAFwdRaw = aFwdRaw;
                if (!Double.isNaN(aFwd))    lastAFwd    = aFwd;

                // Orientation context (logged every row)
                double yawDeg = Double.NaN;
                double tiltDeg = Double.NaN;
                if (hasR) {
                    SensorManager.getOrientation(R, ORI);            // ORI[0]=azimuth (yaw, rad)
                    yawDeg = Math.toDegrees(ORI[0]);
                    // device-Z in world = R*[0,0,1] = (R[2],R[5],R[8]); tilt vs world-Z
                    double cz = clamp(R[8], -1.0, 1.0);
                    tiltDeg = Math.toDegrees(Math.acos(cz));
                }
                double gyroNorm = Math.sqrt(
                        (double)gyrX*(double)gyrX +
                                (double)gyrY*(double)gyrY +
                                (double)gyrZ*(double)gyrZ);

                // CSV row
                String row = String.format(
                        Locale.US,
                        "%.6f,%.6f," +                    // t, dt
                                "%.3f,%.3f,%.3f," +      // ACC
                                "%.5f,%.5f,%.5f," +      // GYR
                                "%.1f,%.1f,%.1f," +      // MAG
                                "%.1f,%.3f,%.1f," +      // yawDeg, gyroNorm, tiltDeg
                                "%.3f,%.3f,%.4f,%.4f",   // aFwd_raw, aFwd, vFwd_raw, vFwd
                        tSec, dtSec,
                        accX, accY, accZ,
                        gyrX, gyrY, gyrZ,
                        magX, magY, magZ,
                        yawDeg, gyroNorm, tiltDeg,
                        aFwdRaw, aFwd, vFwdRaw, vFwd
                );

                if (csv != null) csv.log(row);
                rowsLogged++;

                // UI velocity: show RAW (bias OFF for live)
                binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", vFwdRaw));

                // stats accumulators
                if (Double.isNaN(firstTSec)) firstTSec = tSec;
                lastTSec = tSec;

                if (dtSec > 0) {
                    sumDt += dtSec;
                    nDt += 1.0;
                    if (dtSec < minDt) minDt = dtSec;
                    if (dtSec > maxDt) maxDt = dtSec;
                }

                if (!Double.isNaN(aFwdRaw)) {
                    nARaw++;
                    sumARaw += aFwdRaw;
                    sumA2Raw += aFwdRaw * aFwdRaw;
                    sumAbsARaw += Math.abs(aFwdRaw);
                    absARawList.add(Math.abs(aFwdRaw));

                    if (Double.isNaN(t0Raw)) t0Raw = tSec;
                    double x = tSec - t0Raw;
                    sumXRaw += x; sumX2Raw += x * x; sumYRaw += aFwdRaw; sumXYRaw += x * aFwdRaw;
                }

                if (!Double.isNaN(aFwd)) {
                    nACorr++;
                    sumACorr += aFwd;
                    sumA2Corr += aFwd * aFwd;
                    sumAbsACorr += Math.abs(aFwd);
                    absACorrList.add(Math.abs(aFwd));

                    if (Double.isNaN(t0Corr)) t0Corr = tSec;
                    double x = tSec - t0Corr;
                    sumXCorr += x; sumX2Corr += x * x; sumYCorr += aFwd; sumXYCorr += x * aFwd;
                }
                break;

            case Sensor.TYPE_GYROSCOPE:
                gyrX = event.values[0];
                gyrY = event.values[1];
                gyrZ = event.values[2];
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                magX = event.values[0];
                magY = event.values[1];
                magZ = event.values[2];
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) { }

    // ---------- helpers ----------

    private static void mulR(float[] out, float[] R, float x, float y, float z) {
        out[0] = R[0]*x + R[1]*y + R[2]*z;
        out[1] = R[3]*x + R[4]*y + R[5]*z;
        out[2] = R[6]*x + R[7]*y + R[8]*z;
    }

    private static void normalize3(float[] v) {
        double n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n > 0) {
            v[0] /= n; v[1] /= n; v[2] /= n;
        }
    }

    private static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }

    private String[] buildStatsFooter() {
        // durations & sample rate
        double duration = (!Double.isNaN(firstTSec) && !Double.isNaN(lastTSec)) ? (lastTSec - firstTSec) : 0.0;
        double avgDt = (nDt > 0) ? (sumDt / nDt) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;

        // raw stats
        double meanARaw = (nARaw > 0) ? (sumARaw / nARaw) : Double.NaN;
        double varARaw = (nARaw > 1) ? Math.max(0.0, (sumA2Raw / nARaw) - (meanARaw * meanARaw)) : Double.NaN;
        double stdARaw = (nARaw > 1) ? Math.sqrt(varARaw) : Double.NaN;
        double meanAbsARaw = (nARaw > 0) ? (sumAbsARaw / nARaw) : Double.NaN;

        double p95AbsRaw = Double.NaN;
        if (!absARawList.isEmpty()) {
            Collections.sort(absARawList);
            int m = absARawList.size();
            int idx = (int) Math.floor(0.95 * (m - 1));
            if (idx < 0) idx = 0;
            if (idx >= m) idx = m - 1;
            p95AbsRaw = absARawList.get(idx);
        }

        double slopeRaw = Double.NaN;
        if (nARaw > 1) {
            double N = (double) nARaw;
            double denom = (N * sumX2Raw - sumXRaw * sumXRaw);
            if (Math.abs(denom) > 1e-12) {
                slopeRaw = (N * sumXYRaw - sumXRaw * sumYRaw) / denom;
            }
        }

        // corrected stats
        double meanACorr = (nACorr > 0) ? (sumACorr / nACorr) : Double.NaN;
        double varACorr = (nACorr > 1) ? Math.max(0.0, (sumA2Corr / nACorr) - (meanACorr * meanACorr)) : Double.NaN;
        double stdACorr = (nACorr > 1) ? Math.sqrt(varACorr) : Double.NaN;
        double meanAbsACorr = (nACorr > 0) ? (sumAbsACorr / nACorr) : Double.NaN;

        double p95AbsCorr = Double.NaN;
        if (!absACorrList.isEmpty()) {
            Collections.sort(absACorrList);
            int m = absACorrList.size();
            int idx = (int) Math.floor(0.95 * (m - 1));
            if (idx < 0) idx = 0;
            if (idx >= m) idx = m - 1;
            p95AbsCorr = absACorrList.get(idx);
        }

        double slopeCorr = Double.NaN;
        if (nACorr > 1) {
            double N = (double) nACorr;
            double denom = (N * sumX2Corr - sumXCorr * sumXCorr);
            if (Math.abs(denom) > 1e-12) {
                slopeCorr = (N * sumXYCorr - sumXCorr * sumYCorr) / denom;
            }
        }

        // Build footer lines (after header)
        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- STATS BEGIN ----");
        lines.add(String.format(Locale.US, "# rows_logged=%d", rowsLogged));
        lines.add(String.format(Locale.US, "# duration_s=%.6f", duration));
        lines.add(String.format(Locale.US, "# dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                avgDt, (Double.isInfinite(minDt)?0.0:minDt), maxDt, hz));

        lines.add(String.format(Locale.US, "# aBias=%.6f m/s^2 (calib_n=%d, window_s=%.1f, calibrating=%s)",
                aBias, nBias, CAL_WINDOW_S, Boolean.toString(calibrating)));

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
}
