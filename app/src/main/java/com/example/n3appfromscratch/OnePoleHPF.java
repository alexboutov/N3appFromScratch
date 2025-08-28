package com.example.n3appfromscratch;

/**
 * Simple one-pole high-pass filter with variable dt.
 *
 * Discrete form (from RC high-pass):
 *   y[n] = a * (y[n-1] + x[n] - x[n-1]),  where  a = tau / (tau + dt)
 * tau = 1 / (2*pi*fc)
 */
public class OnePoleHPF {
    private double fcHz;          // cutoff (Hz)
    private double tau;           // time constant (s)
    private boolean hasPrev = false;
    private double xPrev = 0.0;
    private double yPrev = 0.0;

    public OnePoleHPF(double fcHz) {
        setCutoff(fcHz);
    }

    /** Change cutoff without resetting history (call reset() yourself if you need a cold start). */
    public synchronized void setCutoff(double fcHz) {
        this.fcHz = Math.max(1e-6, fcHz);
        this.tau = 1.0 / (2.0 * Math.PI * this.fcHz);
    }

    /** Clear history; next call to filter() will output 0 (no transient bump). */
    public synchronized void reset() {
        hasPrev = false;
        xPrev = 0.0;
        yPrev = 0.0;
    }

    /** Process one sample with its timestep (seconds). */
    public synchronized double filter(double x, double dtSec) {
        if (Double.isNaN(x)) return Double.NaN;
        double dt = Math.max(1e-6, dtSec);

        if (!hasPrev) {
            // initialize quietly
            xPrev = x;
            yPrev = 0.0;
            hasPrev = true;
            return 0.0;
        }

        double a = tau / (tau + dt);
        double y = a * (yPrev + x - xPrev);

        xPrev = x;
        yPrev = y;
        return y;
    }

    public synchronized double getFcHz()  { return fcHz; }
    public synchronized double getTauSec(){ return tau;  }
}
