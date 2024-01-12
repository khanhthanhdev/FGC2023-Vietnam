package org.firstinspires.ftc.teamcode.VNRobot;

//left/right_bumper: intake
// cross/circle : shooter
//        up/down : oxi
//        left/right: wrap, loader
//        square/triangle :
//        left/right bumper: climber

import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.SINK_HEIGHT;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;
import static org.firstinspires.ftc.teamcode.Constants.ODOMETRY.*;

import com.qualcomm.robotcore.util.Range;
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
import org.firstinspires.ftc.teamcode.Subsystems.LiftUp;
import org.firstinspires.ftc.teamcode.Subsystems.Loader;
import org.firstinspires.ftc.teamcode.Subsystems.OxyCascade;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.WrapBall;
import org.firstinspires.ftc.teamcode.utils.Odometry;
import org.firstinspires.ftc.teamcode.utils.ProjectileCalculator;
import org.firstinspires.ftc.teamcode.utils.WheelOdometry;

public class Robot {

    private final Drivebase drivebase;
    private final Intake intake;
    private final Shooter shooter;
    private final Gamepad gamepad1;

    private Telemetry telemetry;
    private final Grab grab;
    private final Loader loader;
    private final WrapBall wrapBall;
    private final OxyCascade oxyCascade;
    private final Climber climber;
    private final LiftUp lift;
    private final IMU imu;
    private final Grab potetiometer;

    private final boolean testMode = false;
    private boolean autoRotateMode = false;
    private Odometry odometry;

    private WheelOdometry TankOdometry;
    boolean shooterState;
    boolean loaderState;
    boolean wrapState;

    public Robot(OpMode opMode) {
        intake = new Intake(opMode);
        drivebase = new Drivebase(opMode);
        shooter = new Shooter(opMode);
        grab = new Grab(opMode);
        loader = new Loader(opMode);
        wrapBall = new WrapBall(opMode);
        lift = new LiftUp(opMode);
        potetiometer = new Grab(opMode);
        oxyCascade = new OxyCascade(opMode);
        climber = new Climber(opMode);
        imu = new IMU(opMode);
        gamepad1 = opMode.gamepad1;
        telemetry = opMode.telemetry;
    }

    public void init() {
        drivebase.init();
        intake.init();
        shooter.init();
        grab.init();
        lift.init();
        potetiometer.init();
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

        TankOdometry = new WheelOdometry(INIT_X,INIT_Y,INIT_THETA);
    }

    public void loop() {

        double intakePower = 0; // intake
        double wrapPower = 0; // wrap
        double OxiLiftUpPower = 0; // lift up oxi storage
        double loaderPower = 0; // loader
        double shooterPower = 0; // shooter
        double climberPower = 0; // climber
        double liftPower = 0; // lift up the robot
        double grabPower = 0;
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_y;
        double leftPower = Range.clip(drive+turn, -1.0, 1.0);
        double rightPower = Range.clip(drive-turn, -1.0, 1.0);


        odometry.update(
                drivebase.getLeftPosition(),
                drivebase.getRightPosition(),
                0,
                new Rotation2d(imu.getYaw())
        );

        TankOdometry.updatePositionWithIMU(drivebase.getLeftPosition(),
                drivebase.getRightPosition(),
                imu.getYaw()
                );

        ProjectileCalculator.update(odometry.getDistance());
        ProjectileCalculator.calculate();



        // Pressing the left stick button will switch between auto rotate mode and manual mode of the robot
        if (gamepad1.touchpad) {
            autoRotateMode = !autoRotateMode;
        }

        // Pressing the right stick button of gamepad 1 will reset the position of the odometry,
        // and start a new series of calculation with new position
        if(gamepad1.right_stick_button) {
            odometry.resetPosition(0, 0, 0);
            TankOdometry.setPose(0,0,0);
        }

        // Pressing the right trigger of the gamepad 1 until it reaches its end will activate the intake
        // While then, pressing the left bumper will boost the speed of intake
        if (gamepad1.right_bumper) {
            intakePower = BOOST_Intake;
            if (gamepad1.left_bumper) {
                intakePower = -intakePower;
            }
        }


        // Run the climber at specific speed
        if (gamepad1.left_trigger > 0.2){
            climberPower = -CLIMBER;
        } else if (gamepad1.right_trigger > 0.2) {
            climberPower = CLIMBER;
        }

        // Run the shooter at a specific speed

        if (gamepad1.cross) {
            shooterState = true;
        }

        if (gamepad1.circle) {
            shooterState = false;
        }

        if (shooterState) {
            shooterPower = shooter.calculate(1200, shooter.getVelocity());

        } else {
            shooterPower = 0;
        }


        // Load ball to shooter
        // Wrap ball in storage
        if (gamepad1.dpad_right) {
            wrapState = true;
            loaderState = true;
        } else if (gamepad1.dpad_left) {
            wrapState = false;
            loaderState = false;
        }
        if (wrapState == true){
            wrapPower = WRAP;
        } else if (wrapState == false) {
            wrapPower = -WRAP;
        }
        if(loaderState == true){
            loaderPower = LOADER;
        } else if (loaderState == false) {
            loaderPower = -LOADER;
        }

        // Lift up O2 storage to accumulator
        if (gamepad1.dpad_up) {
            OxiLiftUpPower = OXYLIFT;
        } else if (gamepad1.dpad_down) {
            OxiLiftUpPower = -OXYLIFT;
        }



        intake.setMotorPower(intakePower);
        loader.load(loaderPower);
        drivebase.setMotorPower(leftPower, rightPower);
        shooter.shoot(-shooterPower);
        wrapBall.wrapSpped(wrapPower);
        oxyCascade.OxyLiftUp(OxiLiftUpPower);
        climber.climb(climberPower);
        lift.Lift(liftPower);
        grab.grabSpeed(grabPower);


        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.addData("Shooter Power", shooter.getMotorPower());

        telemetry.addData("Pose x", odometry.getRobotPose().getX());
        telemetry.addData("Pose y", odometry.getRobotPose().getY());
        telemetry.addData("Heading", odometry.getRobotPose().getHeading());
        telemetry.addData("", TankOdometry.displayPositions());
        telemetry.update();
    }
}