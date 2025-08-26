package com.example.n3appfromscratch;

import java.util.ArrayList;
import java.util.Collections;

class StatsAccumulator {
    // Basic moments
    private long n = 0;
    private double sum = 0.0;
    private double sum2 = 0.0;
    private double sumAbs = 0.0;

    // For percentile
    private final ArrayList<Double> absList = new ArrayList<>();

    // For slope vs time (simple linear regression)
    private double t0 = Double.NaN;
    private double sumX = 0.0, sumX2 = 0.0, sumY = 0.0, sumXY = 0.0;

    void clear() {
        n = 0; sum = 0; sum2 = 0; sumAbs = 0;
        absList.clear();
        t0 = Double.NaN;
        sumX = sumX2 = sumY = sumXY = 0.0;
    }

    void addSample(double tSec, double value) {
        n++;
        sum += value;
        sum2 += value * value;
        double abs = Math.abs(value);
        sumAbs += abs;
        absList.add(abs);

        if (Double.isNaN(t0)) t0 = tSec;
        double x = tSec - t0;
        sumX += x; sumX2 += x*x; sumY += value; sumXY += x*value;
    }

    static class Summary {
        double mean = Double.NaN;
        double std = Double.NaN;
        double meanAbs = Double.NaN;
        double p95Abs = Double.NaN;
        double slope = Double.NaN;
    }

    Summary summarize() {
        Summary s = new Summary();
        if (n > 0) {
            s.mean = sum / n;
            s.meanAbs = sumAbs / n;
        }
        if (n > 1) {
            double var = Math.max(0.0, (sum2 / n) - (s.mean * s.mean));
            s.std = Math.sqrt(var);
        }
        if (!absList.isEmpty()) {
            Collections.sort(absList);
            int m = absList.size();
            int idx = (int) Math.floor(0.95 * (m - 1));
            if (idx < 0) idx = 0;
            if (idx >= m) idx = m - 1;
            s.p95Abs = absList.get(idx);
        }
        if (n > 1) {
            double N = (double) n;
            double denom = (N * sumX2 - sumX * sumX);
            if (Math.abs(denom) > 1e-12) {
                s.slope = (N * sumXY - sumX * sumY) / denom;
            }
        }
        return s;
    }
}
