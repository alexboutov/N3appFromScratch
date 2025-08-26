package com.example.n3appfromscratch;

import android.content.Context;
import android.os.StatFs;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

class CsvLogger {
    private static final String TAG = "Nat3_Sensors";

    private final Context ctx;
    private BufferedWriter out;
    private File dir;
    private File file;
    private String header = "";
    private String baseName = ""; // run_YYYYMMDD_HHMMSS
    private int partIndex = 0;
    private long maxBytes = Long.MAX_VALUE; // rotation threshold; set via setMaxBytes()

    CsvLogger(Context ctx) {
        this.ctx = ctx.getApplicationContext();
    }

    /** Optional: set per-file size limit (bytes). Call before start(). */
    void setMaxBytes(long bytes) {
        this.maxBytes = Math.max(1L, bytes);
    }

    /** Begin a new run with a timestamped base name and write the header. */
    void start(String header) {
        this.header = (header == null) ? "" : header;
        dir = new File(ctx.getExternalFilesDir(null), "logs");
        if (!dir.exists() && !dir.mkdirs()) {
            Log.e(TAG, "Failed to create logs dir: " + dir);
            return;
        }
        baseName = "run_" + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        partIndex = 0;
        rollToNextPart(); // opens _p01 and writes header
        if (file != null) {
            Log.i(TAG, "Logging to: " + file.getAbsolutePath());
        }
    }

    /** Write one CSV row; rotate to a new file if size threshold reached. */
    void log(String row) {
        if (out == null) return;
        try {
            out.write(row);
            out.newLine();
            // lightweight size check; rotate if needed
            if (file != null && file.length() >= maxBytes) {
                rollToNextPart();
            }
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.log error", e);
        }
    }

    /** Finish the run; append header again + optional footer, then close. */
    void finishWithFooter(String repeatHeader, String[] footerLines) {
        if (out == null) return;
        try {
            if (repeatHeader != null && !repeatHeader.isEmpty()) {
                out.write(repeatHeader);
                out.newLine();
            }
            if (footerLines != null) {
                for (String s : footerLines) {
                    out.write(s);
                    out.newLine();
                }
            }
            out.flush();
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.finishWithFooter error", e);
        } finally {
            close();
        }
    }

    void close() {
        try { if (out != null) out.close(); } catch (IOException ignored) {}
        out = null;
        file = null;
    }

    File getFile() { return file; }
    File getDir()  { return dir;  }

    // ---------- Rotation helpers ----------

    private void rollToNextPart() {
        close();
        partIndex++;
        file = new File(dir, baseName + String.format(Locale.US, "_p%02d.csv", partIndex));
        try {
            out = new BufferedWriter(new FileWriter(file, /*append*/ false));
            if (!header.isEmpty()) {
                out.write(header);
                out.newLine();
                out.flush();
            }
            Log.i(TAG, "Switched to: " + file.getAbsolutePath());
        } catch (IOException e) {
            Log.e(TAG, "CsvLogger.rollToNextPart error", e);
            close();
        }
    }

    // ---------- Storage utilities ----------

    static long availableBytes(Context ctx) {
        File base = new File(ctx.getExternalFilesDir(null), ".");
        StatFs fs = new StatFs(base.getAbsolutePath());
        return fs.getAvailableBytes();
    }

    static long totalBytes(Context ctx) {
        File base = new File(ctx.getExternalFilesDir(null), ".");
        StatFs fs = new StatFs(base.getAbsolutePath());
        return fs.getTotalBytes();
    }

    static String human(long bytes) {
        String[] units = {"B","KB","MB","GB","TB"};
        double b = bytes;
        int u = 0;
        while (b >= 1024 && u < units.length-1) { b /= 1024; u++; }
        return new DecimalFormat("#,##0.0").format(b) + " " + units[u];
    }
}
