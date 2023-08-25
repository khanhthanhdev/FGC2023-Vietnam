package org.firstinspires.ftc.teamcode;

/*
    This is the constants file, which provide a convenience place to store consistent data.
    For examples, the speed of a specific device.

 */

public final class Constants {
    // The values in odometry calculation
    public static final class ODOMETRY {
        public static final double INIT_X = 0;
        public static final double INIT_Y = 0;
        public static final double INIT_THETA = 0;

        public static final double ROBOT_WIDTH = 0.5 * 39.3701;
        public static final double ROBOT_LENGTH = 0.48 * 39.3701;
        public static final double ARROW_LENGTH = 2;

        // lf, lb, rf, rb, tracking arrow
        public static final double[] ANGLE_DIRECTIONS =
                {
                        Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH),
                        Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH) + Math.PI / 2,
                        -Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH),
                        -(Math.atan2(ROBOT_WIDTH, ROBOT_LENGTH) + Math.PI / 2),
                        0
                };
        public static final double[] WHEEL_TRANSITIONS =
                {
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        Math.hypot(ROBOT_WIDTH / 2, ROBOT_LENGTH / 2),
                        ARROW_LENGTH
                };
    }

    // The PID constants of shooter's program
    public static final class SHOOT {
        public static final double KP = 1.5;
        public static final double KI = 0;
        public static final double KD = 0;
        public static final double MAX_INTERGRAL = 0;
        public static final double MIN_INTERGRAL = 1;
        public static final double VELOCITY_TOLERANCE = 0.05;
        public static final double POSITION_TOLERANCE = 0.05;
    }
//
//    // The PID constants of robot orientation's program
    public static final class ROTATION {
        public static final double ROTATE_KP = 1.3;
        public static final double ROTATE_KI = 0;
        public static final double ROTATE_KD = 0;
        public static final double TOLERANCE = 0.05;
        public static final double INTERGRAL_MIN = 0;
        public static final double INTERGRAL_MAX = 1;
    }

//     The constants use for projectile calculation, including the position of sink in the coordinate
    public static final class PROJECTILE_MOTION {
        public static final double ANGLE_RATE_ERROR = 0;
        public static final double THETA_ERROR = 0;
        public static final double R1 = 0.03;
        public static final double GRAVITATIONAL_ACCEL = 9.81;
        public static final double SINK_HEIGHT = 1;
        public static final double SINK_X = 4.12; //3.0023
        public static final double SINK_Y = 4; //3.5
        public static final double SINK_RADIUS = 2 * Math.sqrt(3) / 5;
    }
//
    // The PID constants of hood's program
    public static final class HOOD_CONST {
        public static final double HD_KP = 1.3;
        public static final double HD_KI = 0;
        public static final double HD_KD = 0;

        public static final double P_TOLERANCE = 0.08;
        public static final double V_TOLERANCE = 0.05;
    }

    // The speed value in manual controlling
    public static final class SPEED {
        public static final double INTAKE = 0.7;
        public static final double LOADER = -1;
        public static final double WRAP = -1;
        public static final double OXYLIFT = -1;
        public static final double SENTIVITY = 0.2;
        public static final double SHOOTER = 0.5;
        public static final double CLIMBER = 1;
        public static final double BOOST_DrB = 1;
        public static final double NORMAL_DrB = 0.6;
        public static final double BOOST_Intake = -1;
    }
}