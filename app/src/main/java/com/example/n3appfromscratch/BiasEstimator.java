package com.example.n3appfromscratch;

class BiasEstimator {
    private final double windowSec;
    private boolean calibrating = true;
    private long startNs = -1L;

    private double sum = 0.0;
    private long n = 0L;

    private double frozenBias = 0.0;

    BiasEstimator(double windowSec) {
        this.windowSec = Math.max(0.0, windowSec);
        reset();
    }

    void reset() {
        calibrating = true;
        startNs = -1L;
        sum = 0.0;
        n = 0L;
        frozenBias = 0.0;
    }

    void update(long tsNs, double aFwdRaw) {
        if (Double.isNaN(aFwdRaw)) return;
        if (startNs < 0L) startNs = tsNs;

        if (calibrating) {
            sum += aFwdRaw;
            n += 1;
            double elapsed = (tsNs - startNs) / 1_000_000_000.0;
            if (elapsed >= windowSec) {
                frozenBias = (n > 0) ? (sum / n) : 0.0;
                calibrating = false;
            }
        }
    }

    /** Current bias to subtract (running mean if still calibrating, else frozen). */
    double currentBias() {
        if (calibrating) {
            return (n > 0) ? (sum / n) : 0.0;
        }
        return frozenBias;
    }

    boolean isCalibrating() { return calibrating; }
    double getFrozenBias()  { return frozenBias; }
    long   getCalibCount()  { return n; }
    double getWindowSec()   { return windowSec; }
}
