package com.example.n3appfromscratch;

import android.content.Context;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

class CsvLogger {
    private static final String TAG = "Nat3_Sensors";

    private final Context ctx;
    private BufferedWriter out;
    private File file;
    private boolean active = false;

    CsvLogger(Context ctx) {
        this.ctx = ctx.getApplicationContext();
    }

    void start(String header) {
        try {
            File dir = new File(ctx.getExternalFilesDir(null), "logs");
            if (!dir.exists() && !dir.mkdirs()) {
                Log.e(TAG, "Failed to create logs dir: " + dir);
                return;
            }
            String ts = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            file = new File(dir, "run_" + ts + ".csv");
            out = new BufferedWriter(new FileWriter(file, /*append*/ true));
            out.write(header);
            out.newLine();
            out.flush();
            active = true;
            Log.i(TAG, "Logging to: " + file.getAbsolutePath());
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.start error", e);
            active = false;
            close();
        }
    }

    void log(String row) {
        if (!active || out == null) return;
        try {
            out.write(row);
            out.newLine();
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.log error", e);
        }
    }

    void stop(String finalHeader) {
        if (!active) return;
        try {
            if (out != null) {
                out.write(finalHeader);
                out.newLine();
                out.flush();
            }
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.stop error", e);
        } finally {
            active = false;
            close();
        }
    }

    void close() {
        try {
            if (out != null) out.close();
        } catch (IOException ignored) {}
        out = null;
    }

    File getFile() { return file; }
}
