package com.example.n3appfromscratch;

class MathUtil {
    static double clamp(double v, double lo, double hi) {
        return (v < lo) ? lo : (v > hi) ? hi : v;
    }

    static double mag3(double x, double y, double z) {
        return Math.sqrt(x*x + y*y + z*z);
    }

    static double dot3(double ax, double ay, double az, double bx, double by, double bz) {
        return ax*bx + ay*by + az*bz;
    }

    static void normalize3(float[] v) {
        double n = Math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
        if (n > 0) {
            v[0] /= n; v[1] /= n; v[2] /= n;
        }
    }
}
