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

    // ACC-file header (device-forward pipeline)
    private static final String HEADER_ACC =
            "tAcc,dtAcc,accX,accY,accZ,yawDeg,tiltDeg," +
                    "aFwd_raw,aFwd,vFwd_raw,vFwd,distance_vFwd," +
                    "FDU_degrees";

    // LA-file header (azimuth-forward pipeline / legacy P)
    private static final String HEADER_LA =
            "tLa,dtLa,laX,laY,laZ,yawDeg,tiltDeg,azimuthDeg," +
                    "P,vP,distance_vP," +
                    "E_world,N_world,Z_world,azErrDeg";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accRaw, gyro, mag, rotVec, gravity, linAcc;

    // Two independent CSV loggers (separate files, separate clocks)
    private CsvLogger csvAcc = null;
    private CsvLogger csvLa  = null;

    // Motion pipeline
    private MotionPipeline pipeline;
    private final MotionConfig cfg = new MotionConfig(); // default azimuth: 0° (North)

    // Run state
    private boolean isRunning = false;

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
        showIdleVelocities(); // 0.00 m/s in both fields on app launch

        // Tap the info line to set forward azimuth (degrees) for LA pipeline
        binding.tvInfo.setOnClickListener(v -> showAzimuthDialog());

        // START/STOP button
        binding.btnStartStop.setOnClickListener(v -> {
            if (!isRunning) {
                // START both files + reset UI to 0.00 until first updates arrive
                isRunning = true;
                showLiveVelocities(0.0, 0.0); // show 0.00 immediately

                long limitBytes = 10L * 1024 * 1024; // 10 MB per part
                long avail = CsvLogger.availableBytes(this);
                long total = CsvLogger.totalBytes(this);
                Log.i(TAG, "Storage available: " + CsvLogger.human(avail) + " / " + CsvLogger.human(total));
                Log.i(TAG, "Per-file limit: " + CsvLogger.human(limitBytes));

                csvAcc = new CsvLogger(this);
                csvAcc.setMaxBytes(limitBytes);
                csvAcc.setNameSuffix("_acc");      // ACC file tag
                csvAcc.start(HEADER_ACC);

                csvLa = new CsvLogger(this);
                csvLa.setMaxBytes(limitBytes);
                csvLa.setNameSuffix("_la");        // LA file tag
                csvLa.start(HEADER_LA);

                pipeline.start(csvAcc, HEADER_ACC, csvLa, HEADER_LA);
                binding.btnStartStop.setText("STOP");

                Log.i(TAG, "# START ACC" + (csvAcc.getFile()!=null ? " file=" + csvAcc.getFile().getAbsolutePath() : ""));
                Log.i(TAG, "# START LA " + (csvLa .getFile()!=null ? " file=" + csvLa .getFile().getAbsolutePath() : ""));
            } else {
                // STOP both, finalize files
                isRunning = false;

                String[] footerAcc = pipeline.buildAccFooter();
                String[] footerLa  = pipeline.buildLaFooter();

                if (csvAcc != null) {
                    csvAcc.finishWithFooter(HEADER_ACC, footerAcc);
                    Log.i(TAG, HEADER_ACC);
                    for (String s : footerAcc) Log.i(TAG, s);
                    Log.i(TAG, "# STOP ACC" + (csvAcc.getFile()!=null ? " file=" + csvAcc.getFile().getAbsolutePath() : ""));
                }
                if (csvLa != null) {
                    csvLa.finishWithFooter(HEADER_LA, footerLa);
                    Log.i(TAG, HEADER_LA);
                    for (String s : footerLa) Log.i(TAG, s);
                    Log.i(TAG, "# STOP LA " + (csvLa.getFile()!=null ? " file=" + csvLa.getFile().getAbsolutePath() : ""));
                }

                csvAcc = null;
                csvLa  = null;

                // Freeze display at averages of just-completed swim
                double avgAcc = pipeline.getAccAvgVel();
                double avgLa  = pipeline.getLaAvgVel();
                showAverages(avgAcc, avgLa);

                pipeline.stop(); // reset internal flags AFTER we read averages
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
        info += String.format(Locale.US, "  HPF=%.2f Hz  Az=%d° (LA)",
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
                .setMessage("0°=North, 90°=East, 180°=South, 270°=West")
                .setView(input)
                .setPositiveButton("OK", (d, w) -> {
                    try {
                        int val = Integer.parseInt(input.getText().toString().trim());
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

        // If paused while logging, finalize both files and freeze display at current averages
        if (csvAcc != null || csvLa != null) {
            isRunning = false;
            String[] footerAcc = pipeline.buildAccFooter();
            String[] footerLa  = pipeline.buildLaFooter();
            if (csvAcc != null) csvAcc.finishWithFooter(HEADER_ACC, footerAcc);
            if (csvLa  != null) csvLa.finishWithFooter(HEADER_LA,  footerLa);

            double avgAcc = pipeline.getAccAvgVel();
            double avgLa  = pipeline.getLaAvgVel();
            showAverages(avgAcc, avgLa);

            csvAcc = null; csvLa = null;
            pipeline.stop();
            binding.btnStartStop.setText("START");
        }
        Log.i(TAG, "onPause -> unregister");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        pipeline.handleEvent(event);

        // Only update live display while RUNNING
        if (isRunning) {
            double vAcc = pipeline.getUiVelocityAcc();
            double vLa  = pipeline.getUiVelocityLa();
            showLiveVelocities(vAcc, vLa);
        }
    }

    @Override public void onAccuracyChanged(Sensor sensor, int accuracy) { }

    // ---------- UI helpers ----------

    private void showIdleVelocities() {
        binding.tvVelocity.setText(String.format(Locale.US, "ACC 0.00 m/s"));
        if (binding.tvVelocityLa != null) {
            binding.tvVelocityLa.setText(String.format(Locale.US, "LA  0.00 m/s"));
        }
    }

    private void showLiveVelocities(double vAcc, double vLa) {
        binding.tvVelocity.setText(String.format(Locale.US, "ACC %.2f m/s", vAcc));
        if (binding.tvVelocityLa != null) {
            binding.tvVelocityLa.setText(String.format(Locale.US, "LA  %.2f m/s", vLa));
        }
    }

    private void showAverages(double avgAcc, double avgLa) {
        binding.tvVelocity.setText(String.format(Locale.US, "ACC avg %.2f m/s", avgAcc));
        if (binding.tvVelocityLa != null) {
            binding.tvVelocityLa.setText(String.format(Locale.US, "LA  avg %.2f m/s", avgLa));
        }
    }
}
