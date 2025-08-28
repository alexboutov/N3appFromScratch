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

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";

    private static final String HEADER =
            "t,dt,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,yawDeg,gyroNorm,tiltDeg,aFwd_raw,aFwd,vFwd_raw,vFwd,P";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accRaw, gyro, mag, rotVec, gravity, linAcc; // <-- added gravity & linAcc

    // Logging control
    private boolean isLogging = false;

    // File logger
    private CsvLogger csv;

    // Latest device-frame caches
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;   // ACC raw (m/s^2)
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;   // GYR (rad/s)
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;   // MAG (µT)

    // Orientation
    private final float[] R = new float[9];   // device -> world rotation (row-major)
    private final float[] E = new float[3];   // orientation radians: azimuth(yaw), pitch, roll
    private volatile boolean hasR = false;
    private double yawDeg = Double.NaN;       // [-180..180]
    private double yawUnwrapped = Double.NaN; // continuous yaw (deg)
    private double lastYawDeg = Double.NaN;
    private double tiltDeg = Double.NaN;      // 0=vertical, 90=flat

    // Capture-forward-at-START (world-frame forward vector equals device +X at START)
    private boolean fwdCaptured = false;
    private final float[] FWD_WORLD = new float[]{1f, 0f, 0f}; // filled at START from R*[1,0,0]

    // ACC timing
    private long lastAccNs = -1L;

    // Physics
    private static final float G = 9.80665f;  // m/s^2
    private final float[] aW = new float[3];    // world accel incl. g
    private final float[] aWlin = new float[3]; // world accel minus g
    private double aFwdRaw = Double.NaN;        // no bias removal
    private double aFwd = Double.NaN;           // bias-corrected (pre-HPF)
    private double aFwdHPF = Double.NaN;        // high-pass filtered aFwd (used for UI velocity)
    private double lastAFwdRaw = Double.NaN;
    private double lastAFwd = Double.NaN;
    private double lastAFwdHPF = Double.NaN;
    private double vFwdRaw = 0.0;
    private double vFwd = 0.0;                  // UI velocity (integrates HPF(aFwd))

    // Bias calibration (simple mean of aFwdRaw over first CAL_WINDOW_S, then frozen)
    private static final double CAL_WINDOW_S = 3.0;
    private boolean calibrating = false;
    private long calibStartNs = -1L;
    private double sumBias = 0.0;
    private long nBias = 0L;
    private double aBias = 0.0; // final frozen bias

    // -------- HPF (locked to 0.20 Hz for T5) --------
    private static final double[] HPF_FC_PRESETS_HZ = {0.20}; // locked
    private int hpfIndex = 0;
    private OnePoleHPF hpf = new OnePoleHPF(HPF_FC_PRESETS_HZ[0]);

    // -------- Turn-based ZUPT (simple yaw-span detector) --------
    private static final boolean ZUPT_ENABLED = true;
    private static final double TURN_WINDOW_S = 1.0;         // rolling window
    private static final double TURN_YAW_DELTA_DEG = 120.0;  // min yaw span to call it a "turn"
    private static final double ZUPT_MIN_INTERVAL_S = 5.0;   // min gap between ZUPTs
    private static final int   TURN_BUF_MAX = 512;           // cap buffer size
    private double lastZuputTSec = -1.0;
    private int zuptCount = 0;

    private static final class YawSample {
        final double t, yawDegUnwrapped, tiltDeg;
        YawSample(double t, double yaw, double tiltDeg) {
            this.t = t; this.yawDegUnwrapped = yaw; this.tiltDeg = tiltDeg;
        }
    }
    private final Deque<YawSample> yawBuf = new ArrayDeque<>();

    // Stats accumulators (raw and corrected; pre-HPF)
    private long rowsLogged = 0;           // total CSV rows written
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

    // For raw stats
    private long nARaw = 0;                   // count of valid aFwdRaw
    private double sumARaw = 0.0, sumA2Raw = 0.0, sumAbsARaw = 0.0;
    private final ArrayList<Double> absARawList = new ArrayList<>();
    private double t0Raw = Double.NaN, sumXRaw = 0.0, sumX2Raw = 0.0, sumYRaw = 0.0, sumXYRaw = 0.0;

    // For corrected stats (pre-HPF)
    private long nACorr = 0;                   // count of valid aFwd
    private double sumACorr = 0.0, sumA2Corr = 0.0, sumAbsACorr = 0.0;
    private final ArrayList<Double> absACorrList = new ArrayList<>();
    private double t0Corr = Double.NaN, sumXCorr = 0.0, sumX2Corr = 0.0, sumYCorr = 0.0, sumXYCorr = 0.0;

    // --- Legacy P logic state (from old NataDataCollect) ---
    private final float[] latestGravity = new float[3];
    private final float[] latestMag = new float[3];
    private boolean hasGrav = false;
    private boolean hasMag  = false;
    private float latestP = Float.NaN;           // updated on LINEAR_ACCELERATION, logged on ACC
    private static final float POOL_AZIMUTH_DEG = 0f; // due North

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
        gravity= sm.getDefaultSensor(Sensor.TYPE_GRAVITY);                 // added
        linAcc = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);     // added

        String info = "Sensors: "
                + ((accRaw != null) ? "ACC "  : "")
                + ((gyro   != null) ? "GYRO " : "")
                + ((mag    != null) ? "MAG "  : "")
                + ((rotVec != null) ? "RV "   : "")
                + ((gravity!= null) ? "GRAV " : "")
                + ((linAcc != null) ? "LA "   : "");
        binding.tvInfo.setText(info + "  HPF=0.20Hz (locked)");

        // START/STOP
        binding.btnStartStop.setOnClickListener(v -> {
            if (!isLogging) {
                // START
                isLogging = true;

                // reset timing & physics
                lastAccNs = -1L;
                lastAFwdRaw = Double.NaN;
                lastAFwd = Double.NaN;
                lastAFwdHPF = Double.NaN;
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

                // reset HPF
                hpf = new OnePoleHPF(HPF_FC_PRESETS_HZ[hpfIndex]);

                // ZUPT state
                yawBuf.clear();
                lastZuputTSec = -1.0;
                zuptCount = 0;

                // reset P state
                latestP = Float.NaN;
                hasGrav = false;
                hasMag  = false;

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
        if (rotVec  != null) sm.registerListener(this, rotVec,  rate);
        if (accRaw  != null) sm.registerListener(this, accRaw,  rate);
        if (gyro    != null) sm.registerListener(this, gyro,    rate);
        if (mag     != null) sm.registerListener(this, mag,     rate);
        if (gravity != null) sm.registerListener(this, gravity, rate);   // added
        if (linAcc  != null) sm.registerListener(this, linAcc,  rate);   // added
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
        final int type = event.sensor.getType(); // keeping 'final' is fine
        final double tSec = event.timestamp / 1e9;

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR: {
                SensorManager.getRotationMatrixFromVector(R, event.values);
                hasR = true;

                // Orientation angles (radians) -> yawDeg [-180..180]
                SensorManager.getOrientation(R, E);
                double yawNow = Math.toDegrees(E[0]); // azimuth
                yawDeg = yawNow;

                // Unwrap yaw into a continuous signal for span checks
                if (Double.isNaN(yawUnwrapped)) {
                    yawUnwrapped = yawDeg;
                } else {
                    double dy = yawDeg - lastYawDeg;
                    if (dy > 180) dy -= 360;
                    else if (dy < -180) dy += 360;
                    yawUnwrapped += dy;
                }
                lastYawDeg = yawDeg;

                // Tilt (angle between device Z and world Z)
                double cz = R[8]; // world Z component of device Z (R*[0,0,1]) -> out[2]=R[8]
                cz = Math.max(-1.0, Math.min(1.0, cz));
                tiltDeg = Math.toDegrees(Math.acos(cz));
                break;
            }

            case Sensor.TYPE_GRAVITY: { // <-- new: keep latest gravity for P
                latestGravity[0] = event.values[0];
                latestGravity[1] = event.values[1];
                latestGravity[2] = event.values[2];
                hasGrav = true;
                break;
            }

            case Sensor.TYPE_LINEAR_ACCELERATION: { // <-- new: compute P here
                if (hasGrav && hasMag) {
                    latestP = NataDataCollect.computeP(latestGravity, latestMag, event.values, POOL_AZIMUTH_DEG);
                } else {
                    latestP = Float.NaN;
                }
                break;
            }

            case Sensor.TYPE_ACCELEROMETER: {
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];

                // gyroNorm for CSV/UI feel (compute from latest gyro cache)
                double gyroNorm = Math.sqrt(
                        (double)gyrX*gyrX + (double)gyrY*gyrY + (double)gyrZ*gyrZ);

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
                double biasNow = (!calibrating) ? aBias : (nBias > 0 ? (sumBias / nBias) : 0.0);
                aFwd = (!Double.isNaN(aFwdRaw)) ? (aFwdRaw - biasNow) : Double.NaN;

                // High-pass filter for UI velocity
                aFwdHPF = (!Double.isNaN(aFwd)) ? hpf.filter(aFwd, dtSec) : Double.NaN;

                // integrate
                if (!Double.isNaN(aFwdRaw) && !Double.isNaN(lastAFwdRaw) && dtSec > 0) {
                    vFwdRaw += 0.5 * (aFwdRaw + lastAFwdRaw) * dtSec;
                }
                if (!Double.isNaN(aFwdHPF) && !Double.isNaN(lastAFwdHPF) && dtSec > 0) {
                    vFwd += 0.5 * (aFwdHPF + lastAFwdHPF) * dtSec;
                }
                if (!Double.isNaN(aFwdRaw))  lastAFwdRaw  = aFwdRaw;
                if (!Double.isNaN(aFwd))     lastAFwd     = aFwd;
                if (!Double.isNaN(aFwdHPF))  lastAFwdHPF  = aFwdHPF;

                // -------- ZUPT: push current yaw into buffer & check for turn --------
                if (!Double.isNaN(yawUnwrapped)) {
                    yawBuf.addLast(new YawSample(tSec, yawUnwrapped, tiltDeg));
                    if (yawBuf.size() > TURN_BUF_MAX) yawBuf.removeFirst();
                    // prune by window
                    while (!yawBuf.isEmpty() && (tSec - yawBuf.peekFirst().t) > TURN_WINDOW_S) {
                        yawBuf.removeFirst();
                    }
                    checkTurnZUPT(tSec);
                }

                // CSV row (use latestP from LINEAR_ACCELERATION path)
                String row = String.format(
                        Locale.US,
                        "%.6f,%.6f," +            // t, dt
                                "%.3f,%.3f,%.3f," +      // ACC
                                "%.5f,%.5f,%.5f," +      // GYR
                                "%.1f,%.1f,%.1f," +      // MAG
                                "%.1f,%.3f,%.1f," +      // yawDeg, gyroNorm, tiltDeg
                                "%.3f,%.3f,%.4f,%.4f,%.4f",   // aFwd_raw, aFwd, vFwd_raw, vFwd, P
                        tSec, dtSec,
                        accX, accY, accZ,
                        gyrX, gyrY, gyrZ,
                        magX, magY, magZ,
                        yawDeg, gyroNorm, tiltDeg,
                        aFwdRaw, aFwd, vFwdRaw, vFwd, latestP
                );

                if (csv != null) csv.log(row);
                rowsLogged++;

                // update simple UI readout with HPF-integrated velocity
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
            }

            case Sensor.TYPE_GYROSCOPE:
                gyrX = event.values[0];
                gyrY = event.values[1];
                gyrZ = event.values[2];
                break;

            case Sensor.TYPE_MAGNETIC_FIELD:
                magX = event.values[0];
                magY = event.values[1];
                magZ = event.values[2];
                latestMag[0] = event.values[0];
                latestMag[1] = event.values[1];
                latestMag[2] = event.values[2];
                hasMag = true;
                break;
        }
    }

    private void checkTurnZUPT(double tSec) {
        if (!ZUPT_ENABLED) return;
        if (yawBuf.size() < 3) return; // need a few samples

        // Find yaw span in the current window
        double minYaw = Double.POSITIVE_INFINITY, maxYaw = Double.NEGATIVE_INFINITY;
        for (YawSample s : yawBuf) {
            if (s.yawDegUnwrapped < minYaw) minYaw = s.yawDegUnwrapped;
            if (s.yawDegUnwrapped > maxYaw) maxYaw = s.yawDegUnwrapped;
        }
        double span = maxYaw - minYaw;

        if (span >= TURN_YAW_DELTA_DEG) {
            // rate limit
            if (lastZuputTSec < 0.0 || (tSec - lastZuputTSec) >= ZUPT_MIN_INTERVAL_S) {
                vFwd = 0.0;
                vFwdRaw = 0.0;
                lastZuputTSec = tSec;
                zuptCount++;
                Log.i(TAG, String.format(Locale.US,
                        "ZUPT@turn t=%.3f yaw_span=%.1f° (window=%.2fs) -> v=0", tSec, span, TURN_WINDOW_S));
            }
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

    private static void normalize3(float[] v) {
        double n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n > 0) {
            v[0] /= n; v[1] /= n; v[2] /= n;
        }
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

        // corrected stats (pre-HPF)
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

        double fc = HPF_FC_PRESETS_HZ[hpfIndex];
        double tau = 1.0 / (2.0 * Math.PI * fc);

        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- STATS BEGIN ----");
        lines.add(String.format(Locale.US, "# rows_logged=%d", rowsLogged));
        lines.add(String.format(Locale.US, "# duration_s=%.6f", duration));
        lines.add(String.format(Locale.US, "# dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                avgDt, (Double.isInfinite(minDt)?0.0:minDt), maxDt, hz));

        lines.add(String.format(Locale.US, "# aBias=%.6f m/s^2 (calib_n=%d, window_s=%.1f, calibrating=%s)",
                aBias, nBias, CAL_WINDOW_S, Boolean.toString(calibrating)));

        lines.add(String.format(Locale.US, "# HPF_fc_hz=%.3f tau_s=%.3f (UI velocity integrates HPF(aFwd))",
                fc, tau));

        lines.add(String.format(Locale.US, "# ZUPT_turns=%d (window_s=%.2f, min_span_deg=%.0f, min_interval_s=%.1f)",
                zuptCount, TURN_WINDOW_S, TURN_YAW_DELTA_DEG, ZUPT_MIN_INTERVAL_S));

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
