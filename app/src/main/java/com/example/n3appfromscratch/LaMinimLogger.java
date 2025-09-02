package com.example.n3appfromscratch;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Minimal linear-acceleration logger for the "minim" branch.
 * Writes a CSV with header: tLa,dtLa,laX,laY,laZ
 * and a footer with dt statistics and sensor info.
 *
 * Usage (in your MainActivity START/STOP handlers):
 *
 *   // field:
 *   private LaMinimLogger laMinim;
 *
 *   // on START:
 *   if (laMinim == null) laMinim = new LaMinimLogger();
 *   laMinim.start(this, 10L * 1024 * 1024); // 10 MB per file (adjust as desired)
 *
 *   // on STOP:
 *   if (laMinim != null) { laMinim.stop(); laMinim = null; }
 */
public final class LaMinimLogger implements SensorEventListener {

    private static final String TAG = "LaMinimLogger";

    // CSV header
    private static final String HEADER =
            "tLa,dtLa,laX,laY,laZ";

    // File suffix to distinguish from other logs
    private static final String FILE_SUFFIX = "_la_minim";

    // Android
    private SensorManager sm;
    private Sensor laSensor;

    // Logging
    private CsvLogger csv;
    private long maxBytes = 10L * 1024 * 1024; // default 10 MB cap

    // Timebase
    private long   lastLaNs = -1L;
    private long   rows = 0L;
    private double firstTSec = Double.NaN, lastTSec = Double.NaN;
    private double sumDt = 0.0, minDt = Double.POSITIVE_INFINITY, maxDt = 0.0, nDt = 0.0;

    // dt diagnostics
    private long dtLaNegCount = 0L, dtLaZeroCount = 0L;
    private long maxNegDtLaNs = 0L, minPosDtLaNs = Long.MAX_VALUE;

    // sensor info
    private boolean sensorInfoLogged = false;
    private String sensorName = null, sensorVendor = null;
    private int sensorVersion = 0;

    public LaMinimLogger() {}

    /**
     * Begin logging LA to a new CSV.
     * @param ctx Android context
     * @param perFileLimitBytes max bytes per file (CsvLogger will roll)
     */
    public void start(Context ctx, long perFileLimitBytes) {
        stop(); // be safe

        this.maxBytes = perFileLimitBytes > 0 ? perFileLimitBytes : this.maxBytes;

        sm = (SensorManager) ctx.getSystemService(Context.SENSOR_SERVICE);
        laSensor = sm.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        if (laSensor == null) {
            Log.w(TAG, "No TYPE_LINEAR_ACCELERATION sensor on this device.");
            return;
        }

        // create CSV
        csv = new CsvLogger(ctx);
        csv.setFileSuffix(FILE_SUFFIX);
        csv.setMaxBytes(this.maxBytes);
        csv.start(HEADER);

        // register listener
        int rate = SensorManager.SENSOR_DELAY_FASTEST;
        sm.registerListener(this, laSensor, rate);

        // reset stats
        lastLaNs = -1L;
        rows = 0L;
        firstTSec = Double.NaN; lastTSec = Double.NaN;
        sumDt = 0.0; minDt = Double.POSITIVE_INFINITY; maxDt = 0.0; nDt = 0.0;
        dtLaNegCount = 0L; dtLaZeroCount = 0L; maxNegDtLaNs = 0L; minPosDtLaNs = Long.MAX_VALUE;
        sensorInfoLogged = false; sensorName = sensorVendor = null; sensorVersion = 0;

        Log.i(TAG, "LA minim logging STARTED");
    }

    /**
     * Stop logging and write footer.
     */
    public void stop() {
        if (sm != null) {
            sm.unregisterListener(this);
        }
        if (csv != null) {
            csv.finishWithFooter(HEADER, buildFooter());
            csv = null;
            Log.i(TAG, "LA minim logging STOPPED");
        }
        sm = null;
        laSensor = null;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() != Sensor.TYPE_LINEAR_ACCELERATION) return;
        if (csv == null) return; // not started

        if (!sensorInfoLogged) {
            sensorName = event.sensor.getName();
            sensorVendor = event.sensor.getVendor();
            sensorVersion = event.sensor.getVersion();
            sensorInfoLogged = true;
            Log.i(TAG, "LA sensor: " + sensorName + " | vendor=" + sensorVendor + " | v=" + sensorVersion);
        }

        // values
        final float laX = event.values[0];
        final float laY = event.values[1];
        final float laZ = event.values[2];

        // timebase
        final long  nowNs = event.timestamp;
        final double tLa  = nowNs / 1e9;
        final long  dtNs  = (lastLaNs > 0) ? (nowNs - lastLaNs) : 0L;
        final double dtLa = (lastLaNs > 0) ? (dtNs / 1_000_000_000.0) : 0.0;
        lastLaNs = nowNs;

        // dt diagnostics & stats
        if (dtNs == 0L) dtLaZeroCount++;
        else if (dtNs < 0L) {
            dtLaNegCount++;
            long neg = -dtNs;
            if (neg > maxNegDtLaNs) maxNegDtLaNs = neg;
        } else {
            if (dtNs < minPosDtLaNs) minPosDtLaNs = dtNs;
        }

        if (Double.isNaN(firstTSec)) firstTSec = tLa;
        lastTSec = tLa;

        if (dtLa > 0.0) {
            sumDt += dtLa; nDt += 1.0;
            if (dtLa < minDt) minDt = dtLa;
            if (dtLa > maxDt) maxDt = dtLa;
        }

        // CSV row
        final String row = String.format(Locale.US,
                "%.6f,%.6f,%.3f,%.3f,%.3f",
                tLa, dtLa, laX, laY, laZ);
        csv.log(row);
        rows++;
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
        // no-op
    }

    private String[] buildFooter() {
        ArrayList<String> lines = new ArrayList<>();
        lines.add("# ---- LA MINIM STATS ----");
        lines.add(String.format(Locale.US, "# rows=%d", rows));

        final double duration = (!Double.isNaN(firstTSec) && !Double.isNaN(lastTSec))
                ? (lastTSec - firstTSec) : 0.0;
        final double avgDt = (nDt > 0.0) ? (sumDt / nDt) : 0.0;
        final double hz = (avgDt > 0.0) ? (1.0 / avgDt) : 0.0;

        lines.add(String.format(Locale.US,
                "# duration_s=%.6f  dt_avg_s=%.6f  dt_min_s=%.6f  dt_max_s=%.6f  approx_Hz=%.2f",
                duration, avgDt, (Double.isInfinite(minDt) ? 0.0 : minDt), maxDt, hz));

        lines.add(String.format(Locale.US, "# LA sensor: %s | vendor=%s | v=%d",
                String.valueOf(sensorName), String.valueOf(sensorVendor), sensorVersion));

        lines.add(String.format(Locale.US,
                "# dtLa_neg=%d  dtLa_zero=%d  worst_neg_dt_ns=%d  min_pos_dt_ns=%d",
                dtLaNegCount, dtLaZeroCount, maxNegDtLaNs,
                (minPosDtLaNs == Long.MAX_VALUE ? 0 : minPosDtLaNs)));

        return lines.toArray(new String[0]);
    }
}
