package com.example.n3appfromscratch;

class VelocityIntegrator {
    private double v = 0.0;
    private double lastA = Double.NaN;

    void reset() {
        v = 0.0;
        lastA = Double.NaN;
    }

    /** Trapezoid integrate; returns updated velocity. Ignores first sample or nonpositive dt. */
    double update(double dtSec, double a) {
        if (!Double.isFinite(a)) return v;
        if (dtSec > 0.0 && Double.isFinite(lastA)) {
            v += 0.5 * (a + lastA) * dtSec;
        }
        lastA = a;
        return v;
    }

    double value() { return v; }
}
