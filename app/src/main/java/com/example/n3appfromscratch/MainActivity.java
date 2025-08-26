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
import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";

    private static final String HEADER =
            "t,dt,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,yawDeg,gyroNorm,tiltDeg,aFwd_raw,aFwd,vFwd_raw,vFwd";

    // ----- Bias gating thresholds -----
    private static final double BIAS_MIN_QUIET_SEC   = 1.5;
    private static final double BIAS_MAX_SEEK_SEC    = 60.0;   // was 10.0
    private static final double QUIET_GYRO_RAD_S     = 0.05;
    private static final double QUIET_AWLIN_MS2      = 0.10;
    private static final double QUIET_TILT_DEG       = 5.0;

    // ----- High-pass filter for corrected aFwd before integrating vFwd -----
    // y[k] = a * ( y[k-1] + x[k] - x[k-1] ),  a = exp(-2π fc dt)
    private static final double HPF_FC_HZ = 0.12;   // was 0.07
    private double lastAFwd = Double.NaN;           // previous corrected accel (for HPF)
    private double lastAFwdHP = Double.NaN;         // previous HPF output

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accRaw, gyro, mag, rotVec;

    private boolean isLogging = false;
    private CsvLogger csv;

    // device-frame caches
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;

    // orientation helper
    private final OrientationHelper orient = new OrientationHelper();

    // forward vector captured at START (world-frame)
    private boolean fwdCaptured = false;
    private final float[] FWD_WORLD = new float[]{1f, 0f, 0f};

    // timing
    private long lastAccNs = -1L;

    // physics
    private static final float G = 9.80665f;
    private final float[] aW = new float[3];
    private final float[] aWlin = new float[3];
    private double aFwdRaw = Double.NaN;
    private double aFwd = Double.NaN;
    private double lastAFwdRaw = Double.NaN;
    private double vFwdRaw = 0.0;
    private double vFwd = 0.0; // integrates HPF(aFwd)

    // bias calibration (quiet-gated)
    private boolean calibrating = false;
    private double aBias = 0.0;
    private double biasSeekElapsed = 0.0;
    private double quietAccumSec = 0.0;
    private double sumBias = 0.0;
    private long nBias = 0L;

    // stats
    private final StatsAccumulator rawStats  = new StatsAccumulator();
    private final StatsAccumulator corrStats = new StatsAccumulator();

    private long rowsLogged = 0;
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

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

        binding.btnStartStop.setOnClickListener(v -> {
            if (!isLogging) {
                // START
                isLogging = true;

                lastAccNs = -1L;
                lastAFwdRaw = Double.NaN;
                lastAFwd = Double.NaN;
                lastAFwdHP = Double.NaN;
                vFwdRaw = 0.0;
                vFwd = 0.0;

                calibrating = true;
                aBias = 0.0;
                biasSeekElapsed = 0.0;
                quietAccumSec = 0.0;
                sumBias = 0.0;
                nBias = 0L;

                rowsLogged = 0;
                firstTSec = Double.NaN; lastTSec = Double.NaN;
                sumDt = 0.0; minDt = Double.POSITIVE_INFINITY; maxDt = 0.0; nDt = 0.0;
                rawStats.clear();
                corrStats.clear();

                fwdCaptured = false;

                long limitBytes = 10L * 1024 * 1024;
                csv = new CsvLogger(this);
                csv.setMaxBytes(limitBytes);
                csv.start(HEADER);
                Log.i(TAG, HEADER);
                binding.btnStartStop.setText("STOP");
            } else {
                // STOP
                String[] footer = buildStatsFooter();
                if (csv != null) csv.finishWithFooter(HEADER, footer);
                Log.i(TAG, HEADER);
                for (String s : footer) Log.i(TAG, s);
                isLogging = false;
                binding.btnStartStop.setText("START");
                csv = null;
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        int rate = SensorManager.SENSOR_DELAY_FASTEST;
        if (rotVec != null) sm.registerListener(this, rotVec, rate);
        if (accRaw != null) sm.registerListener(this, accRaw, rate);
        if (gyro   != null) sm.registerListener(this, gyro,   rate);
        if (mag    != null) sm.registerListener(this, mag,    rate);
    }

    @Override
    protected void onPause() {
        super.onPause();
        sm.unregisterListener(this);
        if (csv != null) {
            String[] footer = buildStatsFooter();
            csv.finishWithFooter(HEADER, footer);
            csv = null;
        }
        if (isLogging) { isLogging = false; binding.btnStartStop.setText("START"); }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final int type = event.sensor.getType();
        final double tSec = event.timestamp / 1e9;

        switch (type) {
            case Sensor.TYPE_ROTATION_VECTOR:
                orient.updateFromRotationVector(event.values);
                break;

            case Sensor.TYPE_ACCELEROMETER: {
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];
                if (!isLogging) break;

                // dt
                double dtSec;
                if (lastAccNs > 0) dtSec = (event.timestamp - lastAccNs) / 1_000_000_000.0;
                else dtSec = 0.0;
                lastAccNs = event.timestamp;

                // capture forward when R is available
                if (!fwdCaptured && orient.hasR()) {
                    float[] R = orient.getR();
                    FWD_WORLD[0] = R[0];
                    FWD_WORLD[1] = R[3];
                    FWD_WORLD[2] = R[6];
                    MathUtil.normalize3(FWD_WORLD);
                    fwdCaptured = true;
                    Log.i(TAG, String.format(Locale.US,
                            "FWD captured (world): [%.4f, %.4f, %.4f]", FWD_WORLD[0], FWD_WORLD[1], FWD_WORLD[2]));
                }

                double yawDeg = Double.NaN, tiltDeg = Double.NaN;

                if (orient.hasR() && fwdCaptured) {
                    orient.mulR(aW, accX, accY, accZ);
                    aWlin[0] = aW[0];
                    aWlin[1] = aW[1];
                    aWlin[2] = aW[2] - G;

                    aFwdRaw = aWlin[0]*FWD_WORLD[0] + aWlin[1]*FWD_WORLD[1] + aWlin[2]*FWD_WORLD[2];

                    yawDeg  = orient.getYawDeg();
                    tiltDeg = orient.getTiltDeg();
                } else {
                    aFwdRaw = Double.NaN;
                }

                // ---- quiet-gated bias ----
                if (calibrating) {
                    biasSeekElapsed += dtSec;
                    if (!Double.isNaN(aFwdRaw)) {
                        double gyroNorm = Math.sqrt(gyrX*gyrX + gyrY*gyrY + gyrZ*gyrZ);
                        double aWlinNorm = Math.sqrt(aWlin[0]*aWlin[0] + aWlin[1]*aWlin[1] + aWlin[2]*aWlin[2]);
                        double tDeg = (Double.isNaN(tiltDeg) ? 0.0 : tiltDeg);
                        boolean quiet = (gyroNorm < QUIET_GYRO_RAD_S) && (aWlinNorm < QUIET_AWLIN_MS2) && (tDeg < QUIET_TILT_DEG);
                        if (quiet) { sumBias += aFwdRaw; nBias++; quietAccumSec += dtSec; }
                    }
                    if (quietAccumSec >= BIAS_MIN_QUIET_SEC) {
                        aBias = (nBias > 0) ? (sumBias / nBias) : 0.0;
                        calibrating = false;
                        Log.i(TAG, String.format(Locale.US,
                                "Bias frozen (quiet-gated): aBias=%.6f m/s^2 (n=%d, quiet_s=%.2f, seek_s=%.2f)",
                                aBias, nBias, quietAccumSec, biasSeekElapsed));
                    } else if (biasSeekElapsed >= BIAS_MAX_SEEK_SEC) {
                        aBias = 0.0; // safer than a bad bias
                        calibrating = false;
                        Log.i(TAG, String.format(Locale.US,
                                "Bias NOT frozen (insufficient quiet). Using aBias=0.0 (quiet_s=%.2f, seek_s=%.2f)",
                                quietAccumSec, biasSeekElapsed));
                    }
                }

                // corrected accel
                double biasNow = calibrating ? 0.0 : aBias;
                aFwd = (!Double.isNaN(aFwdRaw)) ? (aFwdRaw - biasNow) : Double.NaN;

                // ----- integrate RAW (unchanged) -----
                if (!Double.isNaN(aFwdRaw) && !Double.isNaN(lastAFwdRaw) && dtSec > 0) {
                    vFwdRaw += 0.5 * (aFwdRaw + lastAFwdRaw) * dtSec;
                }
                if (!Double.isNaN(aFwdRaw)) lastAFwdRaw = aFwdRaw;

                // ----- HIGH-PASS aFwd, then integrate to vFwd (corrected) -----
                double aFwdHP = Double.NaN;
                if (!Double.isNaN(aFwd)) {
                    if (Double.isNaN(lastAFwd)) {
                        // first sample: start HPF at zero to avoid a jump
                        aFwdHP = 0.0;
                    } else {
                        double alpha = Math.exp(-2.0 * Math.PI * HPF_FC_HZ * dtSec);
                        double yPrev = Double.isNaN(lastAFwdHP) ? 0.0 : lastAFwdHP;
                        aFwdHP = alpha * ( yPrev + (aFwd - lastAFwd) );
                    }
                }
                if (!Double.isNaN(aFwd)) lastAFwd = aFwd;

                if (!Double.isNaN(aFwdHP) && !Double.isNaN(lastAFwdHP) && dtSec > 0) {
                    vFwd += 0.5 * (aFwdHP + lastAFwdHP) * dtSec;
                }
                if (!Double.isNaN(aFwdHP)) lastAFwdHP = aFwdHP;

                // readouts
                double gyroNormNow = Math.sqrt(gyrX*gyrX + gyrY*gyrY + gyrZ*gyrZ);
                if (Double.isNaN(tiltDeg) && orient.hasR()) tiltDeg = orient.getTiltDeg();

                // CSV row
                String row = String.format(
                        Locale.US,
                        "%.6f,%.6f," +
                                "%.3f,%.3f,%.3f," +
                                "%.5f,%.5f,%.5f," +
                                "%.1f,%.1f,%.1f," +
                                "%.1f,%.3f,%.1f," +
                                "%.3f,%.3f,%.4f,%.4f",
                        tSec, dtSec,
                        accX, accY, accZ,
                        gyrX, gyrY, gyrZ,
                        magX, magY, magZ,
                        yawDeg, gyroNormNow, tiltDeg,
                        aFwdRaw, aFwd, vFwdRaw, vFwd
                );
                if (csv != null) csv.log(row);
                rowsLogged++;

                binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", vFwd));

                // dt stats
                if (Double.isNaN(firstTSec)) firstTSec = tSec;
                lastTSec = tSec;
                if (dtSec > 0) {
                    sumDt += dtSec; nDt += 1.0;
                    if (dtSec < minDt) minDt = dtSec;
                    if (dtSec > maxDt) maxDt = dtSec;
                }

                // a stats (unfiltered)
                if (!Double.isNaN(aFwdRaw)) rawStats.addSample(tSec, aFwdRaw);
                if (!Double.isNaN(aFwd))    corrStats.addSample(tSec, aFwd);
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
                break;
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    private String[] buildStatsFooter() {
        double duration = (!Double.isNaN(firstTSec) && !Double.isNaN(lastTSec)) ? (lastTSec - firstTSec) : 0.0;
        double avgDt = (nDt > 0) ? (sumDt / nDt) : 0.0;
        double hz = (avgDt > 0) ? (1.0 / avgDt) : 0.0;

        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- STATS BEGIN ----");
        lines.add(String.format(Locale.US, "# rows_logged=%d", rowsLogged));
        lines.add(String.format(Locale.US, "# duration_s=%.6f", duration));
        lines.add(String.format(Locale.US, "# dt_avg_s=%.6f dt_min_s=%.6f dt_max_s=%.6f approx_Hz=%.2f",
                avgDt, (Double.isInfinite(minDt)?0.0:minDt), maxDt, hz));

        lines.add(String.format(Locale.US,
                "# aBias=%.6f m/s^2 (calib_n=%d, quiet_s=%.2f, seek_s=%.2f, calibrating=%s)",
                aBias, nBias, quietAccumSec, biasSeekElapsed, Boolean.toString(calibrating)));

        StatsAccumulator.Summary r = rawStats.summarize();
        lines.add(String.format(Locale.US,
                "# RAW  aFwd_mean=%.6f aFwd_std=%.6f aFwd_mean_abs=%.6f aFwd_p95_abs=%.6f",
                r.mean, r.std, r.meanAbs, r.p95Abs));
        lines.add(String.format(Locale.US, "# RAW  aFwd_slope_vs_time=%.8f m/s^3", r.slope));
        lines.add(String.format(Locale.US, "# RAW  vFwd_final=%.6f m/s", vFwdRaw));

        StatsAccumulator.Summary c = corrStats.summarize();
        lines.add(String.format(Locale.US,
                "# CORR aFwd_mean=%.6f aFwd_std=%.6f aFwd_mean_abs=%.6f aFwd_p95_abs=%.6f",
                c.mean, c.std, c.meanAbs, c.p95Abs));
        lines.add(String.format(Locale.US, "# CORR aFwd_slope_vs_time=%.8f m/s^3", c.slope));
        lines.add(String.format(Locale.US, "# CORR vFwd_final=%.6f m/s", vFwd));

        lines.add("# ---- STATS END ----");
        return lines.toArray(new String[0]);
    }
}
