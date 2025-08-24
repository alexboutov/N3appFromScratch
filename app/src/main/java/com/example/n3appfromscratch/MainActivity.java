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

import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";
    private static final String HEADER =
            "t,dt,accX,accY,accZ,linX,linY,linZ,dt_lin,gyrX,gyrY,gyrZ,magX,magY,magZ";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accLin, accRaw, gyro, mag;

    // Logging toggle & timing
    private boolean isLogging = false;
    private double lastTickSec = -1.0;
    private boolean tickOnLinear = true; // prefer LIN as the "tick"; fallback to ACC

    // File logger
    private CsvLogger csv;

    // Latest values cache
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;   // raw accelerometer
    private float linX = Float.NaN, linY = Float.NaN, linZ = Float.NaN;   // linear acceleration
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;   // gyroscope
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;   // magnetometer

    // Linear-accel timing (for dt_lin)
    private long lastLinNs = -1L;
    private double dtLinSec = 0.0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Keep the screen on while this Activity is visible
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        accLin = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        accRaw = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro   = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mag    = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        tickOnLinear = (accLin != null); // if device has LIN, use it as sampling "tick"; else ACC

        String info = "Sensors: "
                + ((accLin!=null)?"LINACC ":"")
                + ((accRaw!=null)?"ACC ":"")
                + ((gyro!=null)?"GYRO ":"")
                + ((mag!=null)?"MAG ":"");
        binding.tvInfo.setText(info);

        // START/STOP toggle
        binding.btnStartStop.setOnClickListener(v -> {
            if (!isLogging) {
                // START
                isLogging = true;
                lastTickSec = -1.0;

                // Decide per-file limit N based on available space
                long avail = CsvLogger.availableBytes(this);
                long total = CsvLogger.totalBytes(this);
                Log.i(TAG, "Storage available: " + CsvLogger.human(avail) + " / " + CsvLogger.human(total));

                // Strategy: use the smaller of 50 MB or 10% of available space, but at least 5 MB
                // long tenPercent = avail / 10;
                // long limitBytes = Math.max(5L * 1024 * 1024, Math.min(50L * 1024 * 1024, tenPercent));
                long limitBytes = 10L * 1024 * 1024; // 25 MB.
                Log.i(TAG, "Using per-file limit: " + CsvLogger.human(limitBytes));

                csv = new CsvLogger(this);
                csv.setMaxBytes(limitBytes);   // <-- rotation threshold
                csv.start(HEADER);             // file header
                Log.i(TAG, HEADER);            // optional: header to Logcat
                binding.btnStartStop.setText("STOP");
                Log.i(TAG, "# START" + (csv != null && csv.getFile()!=null
                        ? " file=" + csv.getFile().getAbsolutePath() : ""));
            } else {
                // STOP
                if (csv != null) {
                    csv.stop(HEADER);          // repeat header in file
                }
                Log.i(TAG, HEADER);            // optional: header to Logcat again
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
        int rate = SensorManager.SENSOR_DELAY_GAME; // ~50 Hz typical
        if (accLin != null) sm.registerListener(this, accLin, rate);
        if (accRaw != null) sm.registerListener(this, accRaw, rate);
        if (gyro   != null) sm.registerListener(this, gyro, rate);
        if (mag    != null) sm.registerListener(this, mag, rate);
        Log.i(TAG, "onResume (listeners registered)");
    }

    @Override
    protected void onPause() {
        super.onPause();
        sm.unregisterListener(this);
        // Be safe: close file if user backgrounds app
        if (csv != null) { csv.close(); csv = null; }
        if (isLogging) { isLogging = false; binding.btnStartStop.setText("START"); }
        Log.i(TAG, "onPause -> unregister");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final int type = event.sensor.getType();
        final double tSec = event.timestamp / 1e9;

        // Update caches
        switch (type) {
            case Sensor.TYPE_ACCELEROMETER:
                accX = event.values[0];
                accY = event.values[1];
                accZ = event.values[2];
                break;
            case Sensor.TYPE_LINEAR_ACCELERATION:
                linX = event.values[0];
                linY = event.values[1];
                linZ = event.values[2];
                // dt_lin: seconds between consecutive LIN events
                if (lastLinNs > 0) {
                    dtLinSec = (event.timestamp - lastLinNs) / 1_000_000_000.0;
                } else {
                    dtLinSec = 0.0;
                }
                lastLinNs = event.timestamp;
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

        if (!isLogging) return;

        // Emit a single CSV row when the "tick" sensor updates
        boolean isTick = (tickOnLinear && type == Sensor.TYPE_LINEAR_ACCELERATION)
                || (!tickOnLinear && type == Sensor.TYPE_ACCELEROMETER);

        if (isTick) {
            double dt = (lastTickSec > 0) ? (tSec - lastTickSec) : 0.0;
            lastTickSec = tSec;

            // If no LIN sensor, keep dt_lin at 0.0
            double dtLinOut = dtLinSec; // already in seconds; format with 5 decimals

            // CSV: t,dt,ACC,LIN,dt_lin,GYR,MAG
            String row = String.format(
                    Locale.US,
                    "%.6f,%.6f," +            // t, dt
                            "%.3f,%.3f,%.3f," +      // ACC ax,ay,az
                            "%.3f,%.3f,%.3f," +      // LIN ax,ay,az
                            "%.5f," +                // dt_lin (s)
                            "%.5f,%.5f,%.5f," +      // GYR gx,gy,gz
                            "%.1f,%.1f,%.1f",        // MAG mx,my,mz
                    tSec, dt,
                    accX, accY, accZ,
                    linX, linY, linZ,
                    dtLinOut,
                    gyrX, gyrY, gyrZ,
                    magX, magY, magZ
            );

            // File + Logcat
            if (csv != null) csv.log(row);
            Log.i(TAG, row);

            // Minimal UI heartbeat
            binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", 0.00));
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Optional: log accuracy changes
    }
}
