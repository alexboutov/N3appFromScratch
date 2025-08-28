package com.example.n3appfromscratch;

import android.hardware.SensorManager;

/**
 * Squeezed by ChatGPT from legacy NataDataCollect.java from 2017.
 * Forward-acceleration "P" using the same logic as the legacy NataDataCollect:
 *  - Use TYPE_GRAVITY + TYPE_MAGNETIC_FIELD to build rotation matrix (device->world).
 *  - Rotate TYPE_LINEAR_ACCELERATION (device frame, gravity-removed) into world frame.
 *  - Project the flat acceleration (E, N) onto the pool azimuth (POC, degrees).
 *
 * World axes (Android): X = East, Y = North, Z = Up.
 * P = E*sin(POC) + N*cos(POC), where POC is pool/forward azimuth in degrees.
 */
public final class NataDataCollect {
    private NataDataCollect() {}

    /**
     * @param gravity   float[3] last TYPE_GRAVITY sample (m/s^2)
     * @param magnetic  float[3] last TYPE_MAGNETIC_FIELD sample (μT)
     * @param linearAcc float[3] current TYPE_LINEAR_ACCELERATION sample (m/s^2)
     * @param poolAzimuthDeg pool/forward azimuth in degrees (0° = North)
     * @return P (forward accel along pool direction), m/s^2; NaN if not computable
     */
    public static float computeP(float[] gravity, float[] magnetic, float[] linearAcc, float poolAzimuthDeg) {
        if (gravity == null || magnetic == null || linearAcc == null) return Float.NaN;
        if (gravity.length < 3 || magnetic.length < 3 || linearAcc.length < 3) return Float.NaN;

        float[] R = new float[9];
        float[] I = new float[9];
        boolean ok = SensorManager.getRotationMatrix(R, I, gravity, magnetic);
        if (!ok) return Float.NaN;

        // world = R * a_device (row-major)
        float E = R[0] * linearAcc[0] + R[1] * linearAcc[1] + R[2] * linearAcc[2]; // world X (East)
        float N = R[3] * linearAcc[0] + R[4] * linearAcc[1] + R[5] * linearAcc[2]; // world Y (North)
        // float U = R[6] * linearAcc[0] + R[7] * linearAcc[1] + R[8] * linearAcc[2]; // Up (unused)

        double rad = Math.toRadians(poolAzimuthDeg);
        return (float)(E * Math.sin(rad) + N * Math.cos(rad));
    }
}
