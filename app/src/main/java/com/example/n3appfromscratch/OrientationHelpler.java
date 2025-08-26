package com.example.n3appfromscratch;

import android.hardware.SensorManager;

class OrientationHelper {
    // Row-major rotation matrix mapping device -> world
    private final float[] R = new float[9];
    private final float[] ori = new float[3]; // azimuth, pitch, roll (radians)
    private boolean hasR = false;

    void updateFromRotationVector(float[] rv) {
        SensorManager.getRotationMatrixFromVector(R, rv);
        hasR = true;
    }

    boolean hasR() { return hasR; }
    float[] getR() { return R; }

    // out = R * [x,y,z]
    void mulR(float[] out, float x, float y, float z) {
        out[0] = R[0]*x + R[1]*y + R[2]*z;
        out[1] = R[3]*x + R[4]*y + R[5]*z;
        out[2] = R[6]*x + R[7]*y + R[8]*z;
    }

    // Azimuth (yaw) in degrees, using SensorManager.getOrientation
    double getYawDeg() {
        if (!hasR) return Double.NaN;
        SensorManager.getOrientation(R, ori);
        return Math.toDegrees(ori[0]); // [-180,180] approx
    }

    // Tilt angle (0 = flat), robust: tilt = acos(worldZ • deviceZ) where deviceZ_in_world = R*[0,0,1] = column 3 -> R[2],R[5],R[8]
    double getTiltDeg() {
        if (!hasR) return Double.NaN;
        double cosTilt = MathUtil.clamp(R[8], -1.0, 1.0); // worldZ component of device Z-axis
        return Math.toDegrees(Math.acos(cosTilt));
    }
}
