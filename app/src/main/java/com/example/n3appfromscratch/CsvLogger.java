package com.example.n3appfromscratch;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Simple CSV logger with part rotation and optional filename suffix.
 * Produces files like:
 *   logs/run_YYYYMMDD_HHmmss[_suffix]_p01.csv
 */
public class CsvLogger {
    private static final String TAG = "CsvLogger";

    private final Context ctx;
    private BufferedWriter out;
    private File currentFile;

    private String header;
    private String runId;          // YYYYMMDD_HHmmss
    private String fileSuffix = ""; // e.g. "_la_minim"
    private int partIndex = 0;

    private long bytesWritten = 0L;
    private long maxBytes = Long.MAX_VALUE;

    public CsvLogger(Context ctx) {
        this.ctx = ctx.getApplicationContext();
    }

    /** Optional: add suffix to filename, e.g., setFileSuffix("_la_minim"). */
    public void setFileSuffix(String suffix) {
        if (suffix == null || suffix.isEmpty()) {
            this.fileSuffix = "";
        } else {
            // Ensure it starts with underscore to separate from timestamp
            this.fileSuffix = suffix.startsWith("_") ? suffix : "_" + suffix;
        }
    }

    /** Optional: set per-file size cap in bytes (will rotate parts). */
    public void setMaxBytes(long maxBytes) {
        if (maxBytes > 0L) this.maxBytes = maxBytes;
    }

    /** Begin a new run with given header (written to each part). */
    public void start(String header) {
        this.header = header != null ? header : "";
        this.runId = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        this.partIndex = 0;
        rotatePart(); // open p01 and write header
        Log.i(TAG, "CSV started: " + (currentFile != null ? currentFile.getAbsolutePath() : "(no file)"));
    }

    /** Append a CSV row (no trailing comma; newline is added). */
    public void log(String row) {
        if (out == null) return;
        try {
            // Rotate if needed (approx; counts UTF-8 bytes)
            int rowBytes = row.getBytes(StandardCharsets.UTF_8).length + 1;
            if (bytesWritten + rowBytes > maxBytes) {
                rotatePart();
            }
            out.write(row);
            out.write('\n');
            bytesWritten += rowBytes;
        } catch (IOException e) {
            Log.w(TAG, "log() failed", e);
        }
    }

    /** Finish current file, writing header again and any footer lines. */
    public void finishWithFooter(String headerToRepeat, String[] footerLines) {
        if (out == null) return;
        try {
            if (headerToRepeat != null && !headerToRepeat.isEmpty()) {
                out.write(headerToRepeat);
                out.write('\n');
            }
            if (footerLines != null) {
                for (String s : footerLines) {
                    out.write(s);
                    out.write('\n');
                }
            }
            out.flush();
            out.close();
        } catch (IOException e) {
            Log.w(TAG, "finishWithFooter() failed", e);
        } finally {
            out = null;
        }
        Log.i(TAG, "CSV finished: " + (currentFile != null ? currentFile.getAbsolutePath() : "(no file)"));
    }

    /** Current file (may be null before start). */
    public File getFile() {
        return currentFile;
    }

    // ---------- internals ----------

    private void rotatePart() {
        closeSilently();
        partIndex++;
        bytesWritten = 0L;

        File dir = resolveLogsDir(ctx);
        if (!dir.exists() && !dir.mkdirs()) {
            Log.w(TAG, "Failed to create logs dir: " + dir.getAbsolutePath());
        }

        String name = String.format(Locale.US,
                "run_%s%s_p%02d.csv", runId, fileSuffix, partIndex);
        currentFile = new File(dir, name);

        try {
            out = new BufferedWriter(new OutputStreamWriter(
                    new FileOutputStream(currentFile, false),
                    StandardCharsets.UTF_8));

            if (header != null && !header.isEmpty()) {
                out.write(header);
                out.write('\n');
                bytesWritten += header.getBytes(StandardCharsets.UTF_8).length + 1;
            }
        } catch (IOException e) {
            Log.w(TAG, "rotatePart() failed", e);
            out = null;
        }
    }

    private static File resolveLogsDir(Context ctx) {
        // Prefer external app-specific dir if available; fallback to internal files/
        File base = ctx.getExternalFilesDir(null);
        if (base == null) base = ctx.getFilesDir();
        return new File(base, "logs");
    }

    private void closeSilently() {
        if (out != null) {
            try {
                out.flush();
                out.close();
            } catch (IOException ignore) {
            } finally {
                out = null;
            }
        }
    }
}
