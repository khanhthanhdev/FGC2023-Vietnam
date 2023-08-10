package org.firstinspires.ftc.teamcode.Odometry;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestRobotPose extends LinearOpMode {

    private MotorEx leftEncoder, rightEncoder;
    private HolonomicOdometry odometry;

    public static final double TRACKWIDTH = 14.31;
    public static final double CENTER_WHEEL_OFFSET = 0.477;
    public static final double WHEEL_DIAMETER = 3.54;

    public static final double TICKS_PER_REV = 560;
    public static final double DISTANCE_PER_PULSE = Math.PI * WHEEL_DIAMETER / TICKS_PER_REV;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new MotorEx(hardwareMap, "leftFront");
        rightEncoder = new MotorEx(hardwareMap, "rightFront");

        leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

        DifferentialOdometry diffOdom = new DifferentialOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_PER_REV,
                () -> rightEncoder.getCurrentPosition() * TICKS_PER_REV,
                TRACKWIDTH
        );



        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            diffOdom.updatePose();

        }
    }
}
