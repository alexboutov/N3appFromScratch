package com.example.n3appfromscratch;

import android.content.Context;
import android.os.Build;
import android.os.Environment;
import android.os.StatFs;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Simple CSV logger with optional file-size rollover and a filename suffix hook.
 * Creates files like: run_YYYYMMDD_HHmmss[_suffix]_pXX.csv in app's external files /logs.
 */
public class CsvLogger {
    private static final String TAG = "CsvLogger";

    private final Context ctx;
    private File logsDir;
    private BufferedWriter bw;
    private File currentFile;

    private long maxBytes = 10L * 1024L * 1024L; // default 10 MB per part; set via setMaxBytes(...)
    private int partIndex = 1;
    private String timestampStamp;               // run-level timestamp (YYYYMMDD_HHmmss)
    private String headerLine = null;
    private boolean started = false;

    // NEW: optional suffix (e.g., "_acc", "_la")
    private String nameSuffix = "";

    public CsvLogger(Context ctx) {
        this.ctx = ctx.getApplicationContext();
        this.logsDir = resolveLogsDir(this.ctx);
        if (!logsDir.exists() && !logsDir.mkdirs()) {
            Log.w(TAG, "Failed to create logs dir: " + logsDir.getAbsolutePath());
        }
    }

    /** Optional: tag the filename, e.g. "_acc" / "_la". */
    public void setNameSuffix(String suffix) {
        this.nameSuffix = (suffix == null) ? "" : suffix.trim();
    }

    /** Optional: set per-file size limit in bytes (<=0 disables rollover). */
    public void setMaxBytes(long maxBytes) {
        this.maxBytes = (maxBytes <= 0) ? Long.MAX_VALUE : maxBytes;
    }

    /** Start a new run; writes the header into the first part. */
    public synchronized void start(String header) {
        this.headerLine = header;
        this.timestampStamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        this.partIndex = 1;
        openNewPart(); // also writes header
        this.started = true;
    }

    /** Write a CSV row (without adding another header). */
    public synchronized void log(String line) {
        if (!started || bw == null) return;
        try {
            // roll if adding this line would exceed limit
            long pendingBytes = (line == null ? 0 : line.getBytes(StandardCharsets.UTF_8).length) + 1; // + newline
            if (currentFile.length() + pendingBytes > maxBytes) {
                // rollover
                closeQuietly();
                partIndex++;
                openNewPart(); // writes header again
            }
            bw.write(line);
            bw.newLine();
        } catch (IOException e) {
            Log.e(TAG, "log() failed", e);
        }
    }

    /**
     * Finish the run: repeat header and append footer lines, then close.
     * If headerRepeat is null, reuses the original headerLine.
     */
    public synchronized void finishWithFooter(String headerRepeat, String[] footerLines) {
        if (!started) return;
        try {
            String hdr = (headerRepeat != null) ? headerRepeat : headerLine;
            if (hdr != null) {
                bw.write(hdr);
                bw.newLine();
            }
            if (footerLines != null) {
                for (String s : footerLines) {
                    if (s == null) continue;
                    bw.write(s);
                    bw.newLine();
                }
            }
        } catch (IOException e) {
            Log.e(TAG, "finishWithFooter() write failed", e);
        } finally {
            closeQuietly();
            started = false;
        }
    }

    /** Returns the current file (may be null before start or after finish). */
    public synchronized File getFile() {
        return currentFile;
    }

    // ---------- helpers ----------

    private void openNewPart() {
        String fname = String.format(Locale.US,
                "run_%s%s_p%02d.csv",
                timestampStamp,
                (nameSuffix.isEmpty() ? "" : nameSuffix),
                partIndex);
        currentFile = new File(logsDir, fname);
        try {
            bw = new BufferedWriter(new FileWriter(currentFile, false));
            if (headerLine != null) {
                bw.write(headerLine);
                bw.newLine();
            }
        } catch (IOException e) {
            Log.e(TAG, "openNewPart() failed", e);
            closeQuietly();
        }
    }

    private void closeQuietly() {
        if (bw != null) {
            try { bw.flush(); } catch (IOException ignored) {}
            try { bw.close(); } catch (IOException ignored) {}
        }
        bw = null;
    }

    private static File resolveLogsDir(Context ctx) {
        File base = ctx.getExternalFilesDir("logs");
        if (base == null) {
            // Fallback to internal app files
            base = new File(ctx.getFilesDir(), "logs");
        }
        return base;
    }

    // ---------- disk info & utilities (static) ----------

    public static long availableBytes(Context ctx) {
        File dir = resolveLogsDir(ctx);
        try {
            StatFs fs = new StatFs(dir.getAbsolutePath());
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2) {
                return fs.getAvailableBytes();
            } else {
                //noinspection deprecation
                return (long) fs.getAvailableBlocks() * (long) fs.getBlockSize();
            }
        } catch (Throwable t) {
            Log.w(TAG, "availableBytes() failed", t);
            return -1L;
        }
    }

    public static long totalBytes(Context ctx) {
        File dir = resolveLogsDir(ctx);
        try {
            StatFs fs = new StatFs(dir.getAbsolutePath());
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.JELLY_BEAN_MR2) {
                return fs.getTotalBytes();
            } else {
                //noinspection deprecation
                return (long) fs.getBlockCount() * (long) fs.getBlockSize();
            }
        } catch (Throwable t) {
            Log.w(TAG, "totalBytes() failed", t);
            return -1L;
        }
    }

    /** Human-readable byte size (e.g., "10.5 MB"). */
    public static String human(long bytes) {
        if (bytes < 0) return "?";
        if (bytes < 1024) return bytes + " B";
        double v = bytes;
        String[] units = {"KB", "MB", "GB", "TB"};
        int i = -1;
        while (v >= 1024 && i < units.length - 1) { v /= 1024; i++; }
        return String.format(Locale.US, "%.1f %s", v, units[i]);
    }
}
