package org.firstinspires.ftc.teamcode.utils;

import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;

// The base calculator for the shooting trajectory.
// This class will return the necessary angle rates and the vertical angle of the hood.
public class ProjectileCalculator {
    private static double d;
    private static double h;
    private static double angleRate = 0;
    private static double theta = 0;

    public static void init(double height) {
        h = height;
    }

    public static void update (double distance) {
        d = distance;
    }

    public static void calculate () {
        double angle = Math.atan(2 * h / d);
        double v0 = Math.sqrt(2 * GRAVITATIONAL_ACCEL * h) / Math.sin(angle);
        angleRate = v0/R1 + ANGLE_RATE_ERROR;
        theta = Math.PI / 2 - angle;
        theta = theta / (2 * Math.PI);
        theta = Math.round(theta * 10) / 10 + THETA_ERROR;
    }

    // Return the angle of the hood
    public static double getAngle() {
        return theta/(1.5*Math.PI);
    }

    // Return the angle rate of the motors.
    public static double getAngleRate() {
        return angleRate;
    }
}