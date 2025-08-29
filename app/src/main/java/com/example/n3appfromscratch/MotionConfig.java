package com.example.n3appfromscratch;

/** Tunable parameters for the motion pipeline. */
public class MotionConfig {
    // Filtering / calibration
    public double HPF_FC_HZ = 0.20;      // kept for ACC pipeline (UI feel if you enable it)
    public double CAL_WINDOW_S = 3.0;

    // ZUPT parameters are unused in this split-logs accuracy pass, kept for future
    public boolean ZUPT_ENABLED = true;
    public double TURN_WINDOW_S = 1.0;
    public double TURN_YAW_DELTA_DEG = 120.0;
    public double ZUPT_MIN_INTERVAL_S = 5.0;

    // Forward direction (degrees, 0° = North) — used by LA pipeline only
    public float poolAzimuthDeg = 0f;

    public MotionConfig setPoolAzimuthDeg(float deg) {
        this.poolAzimuthDeg = deg;
        return this;
    }
}
