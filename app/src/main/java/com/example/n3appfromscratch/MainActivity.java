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
            // ACC tick & dt from ACC; velocity added at the end
            "t,dt,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,aFwd,vFwd";

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
    private final float[] R = new float[9];  // device -> world rotation (row-major)
    private volatile boolean hasR = false;

    // Forward axis (world-frame) hardcoded: South = -Y (Android world: X=East, Y=North, Z=Up)
    //    private static final float[] FWD_SOUTH = new float[]{0f, -1f, 0f};
    // Forward axis (world-frame) hardcoded: East = +X (Android world: X=East, Y=North, Z=Up)
    private static final float[] FWD_SOUTH = new float[]{1f, 0f, 0f};

    // ACC timing
    private long lastAccNs = -1L;

    // Physics
    private static final float G = 9.80665f; // m/s^2
    private final float[] aW = new float[3];    // world accel incl. g
    private final float[] aWlin = new float[3]; // world accel minus g
    private double aFwd = 0.0;
    private double lastAFwd = Double.NaN; // for trapezoid integration
    private double vFwd = 0.0;

    // Stats accumulators (no filtering; desk-test analysis)
    private long rowsLogged = 0;           // total CSV rows written
    private long nA = 0;                   // count of valid aFwd samples
    private double sumA = 0.0, sumA2 = 0.0, sumAbsA = 0.0;
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

    // For slope(aFwd vs time)
    private double t0 = Double.NaN;
    private double sumX = 0.0, sumX2 = 0.0, sumY = 0.0, sumXY = 0.0; // x = t - t0, y = aFwd

    // For percentile of |aFwd|
    private final ArrayList<Double> absAList = new ArrayList<>();

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
        rotVec = sm.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR); // absolute heading; fine for South

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
                lastAFwd = Double.NaN;
                vFwd = 0.0;

                // reset stats
                rowsLogged = 0;
                nA = 0; sumA = 0.0; sumA2 = 0.0; sumAbsA = 0.0;
                firstTSec = Double.NaN; lastTSec = Double.NaN;
                sumDt = 0.0; minDt = Double.POSITIVE_INFINITY; maxDt = 0.0; nDt = 0.0;
                t0 = Double.NaN; sumX = 0.0; sumX2 = 0.0; sumY = 0.0; sumXY = 0.0;
                absAList.clear();

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
                lastAccNs = event.timestamp;

                // compute aFwd using ACC->world - g, projected onto South
                if (hasR) {
                    mulR(aW, R, accX, accY, accZ);   // world accel incl. g
                    aWlin[0] = aW[0];
                    aWlin[1] = aW[1];
                    aWlin[2] = aW[2] - G;            // remove gravity (world Z up)
                    aFwd = aWlin[0]*FWD_SOUTH[0] + aWlin[1]*FWD_SOUTH[1] + aWlin[2]*FWD_SOUTH[2];
                } else {
                    aFwd = Double.NaN;               // no orientation yet
                }

                // integrate velocity (trapezoid) when we have a prior sample and dt>0
                if (!Double.isNaN(aFwd) && !Double.isNaN(lastAFwd) && dtSec > 0) {
                    vFwd += 0.5 * (aFwd + lastAFwd) * dtSec;
                }
                if (!Double.isNaN(aFwd)) lastAFwd = aFwd;

                // CSV row: t,dt,ACC,GYR,MAG,aFwd,vFwd
                String row = String.format(
                        Locale.US,
                        "%.6f,%.6f," +            // t, dt
                                "%.3f,%.3f,%.3f," +      // ACC
                                "%.5f,%.5f,%.5f," +      // GYR
                                "%.1f,%.1f,%.1f," +      // MAG
                                "%.3f,%.4f",             // aFwd, vFwd
                        tSec, dtSec,
                        accX, accY, accZ,
                        gyrX, gyrY, gyrZ,
                        magX, magY, magZ,
                        aFwd, vFwd
                );

                if (csv != null) csv.log(row);
                rowsLogged++;

                // update simple UI readout with velocity
                binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", vFwd));

                // stats accumulators
                if (Double.isNaN(firstTSec)) firstTSec = tSec;
                lastTSec = tSec;

                if (dtSec > 0) {
                    sumDt += dtSec;
                    nDt += 1.0;
                    if (dtSec < minDt) minDt = dtSec;
                    if (dtSec > maxDt) maxDt = dtSec;
                }

                if (!Double.isNaN(aFwd)) {
                    nA++;
                    sumA += aFwd;
                    sumA2 += aFwd * aFwd;
                    sumAbsA += Math.abs(aFwd);
                    absAList.add(Math.abs(aFwd));

                    double x; // time baseline for slope
                    if (Double.isNaN(t0)) t0 = tSec;
                    x = tSec - t0;
                    sumX += x;
                    sumX2 += x * x;
                    sumY += aFwd;
                    sumXY += x * aFwd;
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

    // out = R * [x,y,z], with R 3x3 row-major from SensorManager.getRotationMatrixFromVector
    private static void mulR(float[] out, float[] R, float x, float y, float z) {
        out[0] = R[0]*x + R[1]*y + R[2]*z;
        out[1] = R[3]*x + R[4]*y + R[5]*z;
        out[2] = R[6]*x + R[7]*y + R[8]*z;
    }

    private String[] buildStatsFooter() {
        // durations & sample rate
        double duration = (!Double.isNaN(firstTSec) && !Double.isNaN(lastTSec)) ? (lastTSec - firstTSec) : 0.0;
        double avgDt = (nDt > 0) ? (sumDt / nDt) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;

        // aFwd stats
        double meanA = (nA > 0) ? (sumA / nA) : Double.NaN;
        double varA = (nA > 1) ? Math.max(0.0, (sumA2 / nA) - (meanA * meanA)) : Double.NaN; // population variance
        double stdA = (nA > 1) ? Math.sqrt(varA) : Double.NaN;
        double meanAbsA = (nA > 0) ? (sumAbsA / nA) : Double.NaN;

        double p95Abs = Double.NaN;
        if (!absAList.isEmpty()) {
            Collections.sort(absAList);
            int m = absAList.size();
            int idx = (int) Math.floor(0.95 * (m - 1));
            if (idx < 0) idx = 0;
            if (idx >= m) idx = m - 1;
            p95Abs = absAList.get(idx);
        }

        // slope of aFwd vs time (m/s^3)
        double slope = Double.NaN;
        if (nA > 1) {
            double N = (double) nA;
            double denom = (N * sumX2 - sumX * sumX);
            if (Math.abs(denom) > 1e-12) {
                slope = (N * sumXY - sumX * sumY) / denom;
            }
        }

        // final velocity
        double vEnd = vFwd;

        // Build footer lines (after header)
        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- STATS BEGIN ----");
        lines.add(String.format(Locale.US, "# rows_logged=%d", rowsLogged));
        lines.add(String.format(Locale.US, "# duration_s=%.6f", duration));
        lines.add(String.format(Locale.US, "# dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                avgDt, (Double.isInfinite(minDt)?0.0:minDt), maxDt, hz));
        lines.add(String.format(Locale.US, "# aFwd_mean=%.6f aFwd_std=%.6f aFwd_mean_abs=%.6f aFwd_p95_abs=%.6f",
                meanA, stdA, meanAbsA, p95Abs));
        lines.add(String.format(Locale.US, "# aFwd_slope_vs_time=%.8f m/s^3", slope));
        lines.add(String.format(Locale.US, "# vFwd_final=%.6f m/s", vEnd));
        lines.add("# ---- STATS END ----");
        return lines.toArray(new String[0]);
    }
}
