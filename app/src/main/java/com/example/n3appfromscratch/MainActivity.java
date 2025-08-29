package com.example.n3appfromscratch;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.os.Bundle;
import android.text.InputType;
import android.util.Log;
import android.view.WindowManager;
import android.widget.EditText;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import com.example.n3appfromscratch.databinding.ActivityMainBinding;

import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";

    // Header includes "P" at the end (forward accel along pool azimuth)
    private static final String HEADER =
            "t,dt,accX,accY,accZ,gyrX,gyrY,gyrZ,magX,magY,magZ,yawDeg,gyroNorm,tiltDeg,aFwd_raw,aFwd,vFwd_raw,vFwd,P";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accRaw, gyro, mag, rotVec, gravity, linAcc;

    // Logging
    private CsvLogger csv;

    // Motion pipeline (all logic/state lives here)
    private MotionPipeline pipeline;
    private final MotionConfig cfg = new MotionConfig(); // default: azimuth 0° (North)

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
        gravity= sm.getDefaultSensor(Sensor.TYPE_GRAVITY);
        linAcc = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);

        pipeline = new MotionPipeline(cfg);

        refreshInfoLabel();

        // Tap the info line to set forward azimuth (degrees)
        binding.tvInfo.setOnClickListener(v -> showAzimuthDialog());

        // START/STOP button
        binding.btnStartStop.setOnClickListener(v -> {
            if (csv == null) {
                // START
                long limitBytes = 10L * 1024 * 1024; // 10 MB per part
                long avail = CsvLogger.availableBytes(this);
                long total = CsvLogger.totalBytes(this);
                Log.i(TAG, "Storage available: " + CsvLogger.human(avail) + " / " + CsvLogger.human(total));
                Log.i(TAG, "Using per-file limit: " + CsvLogger.human(limitBytes));

                csv = new CsvLogger(this);
                csv.setMaxBytes(limitBytes);

                pipeline.startLogging(csv, HEADER);  // pipeline starts CSV & logs header
                binding.btnStartStop.setText("STOP");
                Log.i(TAG, "# START" + (csv.getFile() != null ? " file=" + csv.getFile().getAbsolutePath() : ""));
            } else {
                // STOP
                String[] footer = pipeline.stopAndGetFooter();
                csv.finishWithFooter(HEADER, footer);
                Log.i(TAG, HEADER);
                for (String s : footer) Log.i(TAG, s);

                Log.i(TAG, "# STOP" + (csv.getFile() != null ? " file=" + csv.getFile().getAbsolutePath() : ""));
                csv = null;
                binding.btnStartStop.setText("START");
            }
        });
    }

    private void refreshInfoLabel() {
        String info = "Sensors: "
                + ((accRaw != null) ? "ACC "  : "")
                + ((gyro   != null) ? "GYRO " : "")
                + ((mag    != null) ? "MAG "  : "")
                + ((rotVec != null) ? "RV "   : "")
                + ((gravity!= null) ? "GRAV " : "")
                + ((linAcc != null) ? "LA "   : "");
        info += String.format(Locale.US, "  HPF=%.2f Hz  Az=%d°",
                cfg.HPF_FC_HZ, Math.round(cfg.poolAzimuthDeg));
        binding.tvInfo.setText(info);
    }

    private void showAzimuthDialog() {
        final EditText input = new EditText(this);
        input.setInputType(InputType.TYPE_CLASS_NUMBER | InputType.TYPE_NUMBER_FLAG_SIGNED);
        input.setHint("0..359");
        input.setText(String.valueOf(Math.round(cfg.poolAzimuthDeg)));

        new AlertDialog.Builder(this)
                .setTitle("Set Pool Azimuth (deg)")
                .setMessage("0° = North, 90° = East, 180° = South, 270° = West")
                .setView(input)
                .setPositiveButton("OK", (d, w) -> {
                    try {
                        int val = Integer.parseInt(input.getText().toString().trim());
                        // Normalize to [0,360)
                        int norm = ((val % 360) + 360) % 360;
                        pipeline.setPoolAzimuthDeg(norm);
                        refreshInfoLabel();
                    } catch (Exception ignored) { }
                })
                .setNegativeButton("Cancel", null)
                .show();
    }

    @Override
    protected void onResume() {
        super.onResume();
        int rate = SensorManager.SENSOR_DELAY_FASTEST;
        if (rotVec  != null) sm.registerListener(this, rotVec,  rate);
        if (accRaw  != null) sm.registerListener(this, accRaw,  rate);
        if (gyro    != null) sm.registerListener(this, gyro,    rate);
        if (mag     != null) sm.registerListener(this, mag,     rate);
        if (gravity != null) sm.registerListener(this, gravity, rate);
        if (linAcc  != null) sm.registerListener(this, linAcc,  rate);
        Log.i(TAG, "onResume (listeners registered, FASTEST)");
    }

    @Override
    protected void onPause() {
        super.onPause();
        sm.unregisterListener(this);
        if (csv != null) {
            // finalize logging to avoid a dangling file
            String[] footer = pipeline.stopAndGetFooter();
            csv.finishWithFooter(HEADER, footer);
            csv = null;
        }
        binding.btnStartStop.setText("START");
        Log.i(TAG, "onPause -> unregister");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        pipeline.handleEvent(event);

        // Update the velocity text when ACC events come in (fast, steady cadence)
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", pipeline.getUiVelocity()));
        }
    }

    @Override public void onAccuracyChanged(Sensor sensor, int accuracy) { }
}
