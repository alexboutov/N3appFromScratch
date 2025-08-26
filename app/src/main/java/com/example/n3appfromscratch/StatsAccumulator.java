package com.example.n3appfromscratch;

import java.util.ArrayList;
import java.util.Collections;

class StatsAccumulator {
    private long n = 0;
    private double sum = 0.0, sum2 = 0.0, sumAbs = 0.0;
    private final ArrayList<Double> absList = new ArrayList<>();

    // For slope(a vs time) using least squares
    private double t0 = Double.NaN, sumX = 0.0, sumX2 = 0.0, sumY = 0.0, sumXY = 0.0;

    void reset() {
        n = 0; sum = 0; sum2 = 0; sumAbs = 0;
        absList.clear();
        t0 = Double.NaN; sumX = 0; sumX2 = 0; sumY = 0; sumXY = 0;
    }

    void add(double tSec, double a) {
        if (!Double.isFinite(a)) return;
        n += 1;
        sum += a;
        sum2 += a * a;
        double aa = Math.abs(a);
        sumAbs += aa;
        absList.add(aa);

        if (Double.isNaN(t0)) t0 = tSec;
        double x = tSec - t0;
        sumX += x; sumX2 += x * x; sumY += a; sumXY += x * a;
    }

    Summary summary() {
        double mean = (n > 0) ? sum / n : Double.NaN;
        double var  = (n > 1) ? Math.max(0.0, (sum2 / n) - mean * mean) : Double.NaN;
        double std  = (n > 1) ? Math.sqrt(var) : Double.NaN;
        double meanAbs = (n > 0) ? sumAbs / n : Double.NaN;

        double p95 = Double.NaN;
        if (!absList.isEmpty()) {
            Collections.sort(absList);
            int m = absList.size();
            int idx = (int)Math.floor(0.95 * (m - 1));
            if (idx < 0) idx = 0; if (idx >= m) idx = m - 1;
            p95 = absList.get(idx);
        }

        double slope = Double.NaN;
        if (n > 1) {
            double N = (double)n;
            double denom = (N * sumX2 - sumX * sumX);
            if (Math.abs(denom) > 1e-12) {
                slope = (N * sumXY - sumX * sumY) / denom;
            }
        }
        return new Summary(n, mean, std, meanAbs, p95, slope);
    }

    static final class Summary {
        final long n;
        final double mean, std, meanAbs, p95Abs, slope;
        Summary(long n, double mean, double std, double meanAbs, double p95Abs, double slope) {
            this.n = n; this.mean = mean; this.std = std; this.meanAbs = meanAbs; this.p95Abs = p95Abs; this.slope = slope;
        }
    }
}
