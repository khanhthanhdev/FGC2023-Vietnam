package org.firstinspires.ftc.teamcode.VNRobot;

import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.SINK_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Climber;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Grab;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Loader;
import org.firstinspires.ftc.teamcode.Subsystems.LoaderGate;
import org.firstinspires.ftc.teamcode.Subsystems.OxyCascade;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WrapBall;
import org.firstinspires.ftc.teamcode.utils.Odometry;
import org.firstinspires.ftc.teamcode.utils.ProjectileCalculator;

public class Robot {

    private final Drivebase drivebase;
    private final Intake intake;
    private final Shooter shooter;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    private Telemetry telemetry;
    private final Grab grab;
    private final LoaderGate loaderGate;
    private final Loader loader;
    private final WrapBall wrapBall;
    private final OxyCascade oxyCascade;
    private final Climber climber;
    private final IMU imu;

    private final boolean testMode = false;
    private boolean autoRotateMode = false;
    private Odometry odometry;

    double gatePosition;
    boolean oldGatePosition;

//    double grabPosition;
//
//    boolean oldGrabPosition;
    boolean shooterState;

    public Robot(OpMode opMode) {
        intake = new Intake(opMode);
        drivebase = new Drivebase(opMode);
        shooter = new Shooter(opMode);
        grab = new Grab(opMode);
        loaderGate = new LoaderGate(opMode);
        loader = new Loader(opMode);
        wrapBall = new WrapBall(opMode);
        oxyCascade = new OxyCascade(opMode);
        climber = new Climber(opMode);
        imu = new IMU(opMode);
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        telemetry = opMode.telemetry;
    }

    public void init() {
        drivebase.init();
        intake.init();
        shooter.init();
        grab.init();
        loaderGate.init();
        loader.init();
        wrapBall.init();
        oxyCascade.init();
        climber.init();
        imu.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ProjectileCalculator.init(SINK_HEIGHT);
    }

    public void start() {
        odometry = new Odometry(
                new Pose2d(INIT_X, INIT_Y, new Rotation2d(INIT_THETA)),
                new Rotation2d(imu.getYaw())
        );
    }

    public void loop() {

        double intakePower = 0; // intake
        double wrapPower = 0; // wrap
        double OxiLiftUpPower = 0; // lift up oxi storage
        double loaderPower = 0; // loader
        double shooterPower = 0; // shooter
        double climberPower = 0; // climber
        double servoSentivity = SENTIVITY;
        double grabPower = 0.5;
        double MAX_SPEED = NORMAL_DrB;
        double leftDrB = 0;
        double rightDrB = 0;

        boolean gateButton = gamepad2.square;
//        boolean grabButton = gamepad1.triangle;

        odometry.update(
                drivebase.getLeftPosition(),
                drivebase.getRightPosition(),
                0,
                new Rotation2d(imu.getYaw())
        );

        ProjectileCalculator.update(odometry.getDistance());
        ProjectileCalculator.calculate();

        // The main mode to run the robot

        // Pressing RB will boost the speed of the chassis
        if (gamepad1.left_trigger > 0.5) {
            MAX_SPEED = BOOST_DrB;
        }

        // Control the speed of the chassis by 2 joysticks on the gamepad
        leftDrB = gamepad1.left_stick_y * MAX_SPEED;
        rightDrB = gamepad1.right_stick_y * MAX_SPEED;

        if (Math.abs(gamepad2.right_stick_y) >= 0.2){
            grab.grabSpeed((gamepad2.right_stick_y*servoSentivity) + grabPower);
        }




        // Pressing the left stick button will switch between auto rotate mode and manual mode of the robot
        if (gamepad1.touchpad) {
            autoRotateMode = !autoRotateMode;
        }
        if (autoRotateMode){
            leftDrB = -leftDrB;
            rightDrB = -rightDrB;
        }

        // Pressing the right stick button of gamepad 1 will reset the position of the odometry,
        // and start a new series of calculation with new position
        //            if(gamepad1.right_stick_button) {
        //                odometry.resetPosition(0, 0, 0);
        //            }

        // Pressing the right trigger of the gamepad 1 until it reaches its end will activate the intake
        // While then, pressing the left bumper will boost the speed of intake
        if (gamepad1.right_bumper) {
            intakePower = INTAKE;
            if (gamepad1.right_trigger > 0.5) {
                intakePower = BOOST_Intake;
            }
        }



        // Run the climber at specific speed
        if (gamepad2.right_trigger > 0.5) {
            climberPower = CLIMBER;
        }

        // Run the shooter at a specific speed

        if (gamepad2.cross) {
            shooterState = true;
        }

        if (gamepad2.circle) {
            shooterState = false;
        }

        if (shooterState) {
            shooterPower = shooter.calculate(1200, shooter.getVelocity());
        } else {
            shooterPower = 0;
        }

        if (gamepad1.left_bumper) {
            intakePower = -intakePower;
        }

        // While the devices are working, Triggering the L2 of gamepad 2 will reverse the direction of all of them. (in their manual mode)
        if (gamepad2.left_trigger > 0.5) {
            shooterPower = -shooterPower;
            climberPower = -climberPower;
        }

        // Open Gate for the ball go to shooter
        if (gateButton && !oldGatePosition) {
            if (gatePosition == 0) {
                loaderGate.OpenGate(0.7);
                gatePosition = 1;
            } else {
                loaderGate.OpenGate(0);
                gatePosition = 0;
            }
        }

        oldGatePosition = gateButton;

        // Hold Hidro tank
//        if (grabButton && !oldGrabPosition) {
//            if (grabPosition == 0) {
//                grab.grabPos(0.7);
//                grabPosition = 1;
//            } else {
//                grab.grabPos(0);
//                grabPosition = 0;
//            }
//        }
//
//        oldGrabPosition = grabButton;

        // Load ball to shooter
        if (gamepad2.dpad_up) {
            loaderPower = LOADER;
        } else if (gamepad2.dpad_down) {
            loaderPower = -LOADER;
        }

        // Wrap ball in storage
        if (gamepad2.dpad_right) {
            wrapPower = WRAP;
        } else if (gamepad2.dpad_left) {
            wrapPower = -WRAP;
        }

        // Lift up O2 storage to accumulator
        if (gamepad2.right_bumper) {
            OxiLiftUpPower = OXYLIFT;
        } else if (gamepad2.left_bumper) {
            OxiLiftUpPower = -OXYLIFT;
        }

        intake.setMotorPower(intakePower);
        loader.load(loaderPower);
        drivebase.setMotorPower(rightDrB, leftDrB);
        shooter.shoot(shooterPower);
        wrapBall.wrapSpped(wrapPower);
        oxyCascade.OxyLiftUp(OxiLiftUpPower);
        climber.climb(climberPower);


        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.addData("Shooter Power", shooter.getMotorPower());
        telemetry.addData("Intake speed", intakePower);
//        telemetry.addData("Loader state", loaderPower);

        telemetry.addData("left DrB speed", leftDrB);
        telemetry.addData("Right DrB speed", rightDrB);
//        telemetry.addData("Max DrB speed", MAX_SPEED);
//
        telemetry.addData("Grab Speed", grabPower);
        telemetry.addData("Climber speed", climberPower);
        telemetry.addData("Pose x", odometry.getRobotPose().getX());
        telemetry.addData("Pose y", odometry.getRobotPose().getY());
        telemetry.addData("Heading", odometry.getRobotPose().getHeading());
        telemetry.update();
    }
}