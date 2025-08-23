package com.example.n3appfromscratch;

import android.os.Bundle;
import android.util.Log;
import androidx.appcompat.app.AppCompatActivity;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.content.Context;
import android.view.WindowManager;

import com.example.n3appfromscratch.databinding.ActivityMainBinding;

import java.util.Locale;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    private static final String TAG = "Nat3_Sensors";

    private ActivityMainBinding binding;
    private SensorManager sm;
    private Sensor accLin, accRaw, gyro, mag;

    // Logging toggle & timing
    private boolean isLogging = false;
    private double lastTickSec = -1.0;
    private boolean tickOnLinear = true; // prefer LIN as the "tick"; fallback to ACC if LIN missing

    // Latest values cache (used to print all columns every tick)
    private float accX = Float.NaN, accY = Float.NaN, accZ = Float.NaN;   // raw accelerometer
    private float linX = Float.NaN, linY = Float.NaN, linZ = Float.NaN;   // linear acceleration
    private float gyrX = Float.NaN, gyrY = Float.NaN, gyrZ = Float.NaN;   // gyroscope
    private float magX = Float.NaN, magY = Float.NaN, magZ = Float.NaN;   // magnetometer

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        binding = ActivityMainBinding.inflate(getLayoutInflater());
        setContentView(binding.getRoot());

        // Keep screen on during tests
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        sm = (SensorManager) getSystemService(Context.SENSOR_SERVICE);

        accLin = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        accRaw = sm.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        gyro   = sm.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        mag    = sm.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        tickOnLinear = (accLin != null); // if device has LIN, use it as sampling "tick"; else use ACC

        String info = "Sensors: "
                + ((accLin!=null)?"LINACC ":"")
                + ((accRaw!=null)?"ACC ":"")
                + ((gyro!=null)?"GYRO ":"")
                + ((mag!=null)?"MAG ":"");
        binding.tvInfo.setText(info);

        // Toggle START/STOP
        binding.btnStartStop.setOnClickListener(v -> {
            if (!isLogging) {
                // START
                isLogging = true;
                lastTickSec = -1.0;
                printHeader();
                binding.btnStartStop.setText("STOP");
                Log.i(TAG, "# START");
            } else {
                // STOP
                printHeader(); // repeat header on STOP as requested
                isLogging = false;
                binding.btnStartStop.setText("START");
                Log.i(TAG, "# STOP");
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
        Log.i(TAG, "onPause -> unregister");
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        final int type = event.sensor.getType();
        final double tSec = event.timestamp / 1e9;

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

            // CSV: t,dt,ACC,LIN,GYR,MAG (14 columns total)
            String row = String.format(
                    Locale.US,
                    "%.6f,%.6f," +   // t, dt
                            "%.3f,%.3f,%.3f," +   // ACC ax,ay,az
                            "%.3f,%.3f,%.3f," +   // LIN ax,ay,az
                            "%.5f,%.5f,%.5f," +   // GYR gx,gy,gz
                            "%.1f,%.1f,%.1f",     // MAG mx,my,mz
                    tSec, dt,
                    accX, accY, accZ,
                    linX, linY, linZ,
                    gyrX, gyrY, gyrZ,
                    magX, magY, magZ
            );
            Log.d(TAG, row);

            // Minimal UI heartbeat
            binding.tvVelocity.setText(String.format(Locale.US, "%.2f m/s", 0.00));
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Optional: log accuracy changes
    }

    // Header once on START, and again on STOP per your request
    private void printHeader() {
        Log.d(TAG, "t,dt,accX,accY,accZ,linX,linY,linZ,gyrX,gyrY,gyrZ,magX,magY,magZ");
    }
}
