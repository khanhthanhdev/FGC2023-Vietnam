package org.firstinspires.ftc.teamcode.VNRobot;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

import org.firstinspires.ftc.teamcode.Subsystems.Grab;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Loader;
import org.firstinspires.ftc.teamcode.Subsystems.LoaderGate;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.StreamCamera;
import org.firstinspires.ftc.teamcode.utils.ProjectileCalculator;



import static org.firstinspires.ftc.teamcode.Constants.PROJECTILE_MOTION.*;
import static org.firstinspires.ftc.teamcode.Constants.SPEED.*;

public class Robot {

    private Drivebase drivebase;
    private Intake intake;
    private Shooter shooter;
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private Telemetry telemetry;
    private Grab grab;
    private LoaderGate loaderGate;
    private Loader loader;

    private StreamCamera webcam;

    private  boolean testMode = false;
    private boolean autoRotateMode = false;

    double gatePosition;
    boolean oldGatePosition;

    double grabPosition;

    boolean oldGrabPosition;
    boolean shooterState;



    public  Robot(OpMode opMode){
        intake = new Intake(opMode);
        drivebase = new Drivebase(opMode);
        shooter = new Shooter(opMode);
        grab = new Grab(opMode);
        loaderGate = new LoaderGate(opMode);
        loader = new Loader(opMode);
        webcam = new StreamCamera(opMode);

        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;
        telemetry = opMode.telemetry;
    }

    public void init(){
        drivebase.init();
        intake.init();
        shooter.init();
        grab.init();
        loaderGate.init();
        loader.init();
        webcam.init();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        ProjectileCalculator.init(SINK_HEIGHT);
    }

    public void loop(){

        double ikV = 0;
        double ldV = 0;
        double stV = 0;
        double clbV = 0;
        double MAX_SPEED = NORMAL_DrB;
        double leftDrB = 0;
        double rightDrB = 0;

        boolean gateButton = gamepad2.square;
        boolean grabButton = gamepad1.triangle;


        ProjectileCalculator.calculate();



        // Trigger by left bumper of gamepad 2, switching between test and main automatically after each time pressing LB
        if(gamepad2.left_bumper) {
            testMode =! testMode;
        }

        if(testMode) {
            // Only one mechanism can be tested at a time
            if(gamepad1.cross) {
                stV = gamepad1.left_stick_y;
            }

            else if(gamepad1.circle) {
                ikV = gamepad1.left_stick_y;
            }

            else {
                MAX_SPEED = gamepad1.left_trigger;
                leftDrB = gamepad1.left_stick_y * MAX_SPEED;
                rightDrB = gamepad1.right_stick_y * MAX_SPEED;
            }
        }

        // The main mode to run the robot
        else {
            // Pressing RB will boost the speed of the chassis
            if (gamepad1.left_trigger > 0.7) {
                MAX_SPEED = BOOST_DrB;
            }

            // Control the speed of the chassis by 2 joysticks on the gamepad
            leftDrB = gamepad1.left_stick_y * MAX_SPEED;
            rightDrB = gamepad1.right_stick_y * MAX_SPEED;

            // Pressing the left stick button will switch between auto rotate mode and manual mode of the robot
            if (gamepad1.touchpad) {
                autoRotateMode = !autoRotateMode;
            }

            // Pressing the right stick button of gamepad 1 will reset the position of the odometry,
            // and start a new series of calculation with new position
//            if(gamepad1.right_stick_button) {
//                odometry.resetPosition(0, 0, 0);
//            }

            // Pressing the right trigger of the gamepad 1 until it reaches its end will activate the intake
            // While then, pressing the left bumper will boost the speed of intake
            if (gamepad1.right_bumper) {
                ikV = INTAKE;
                if (gamepad1.right_trigger > 0.7) {
                    ikV = BOOST_Intake;
                }
            }

            // Run the loader at specific speed

            // Run the climber at specific speed
            if (gamepad2.right_trigger > 0.8) {
                clbV = CLIMBER;
            }

            // Run the shooter at a specific speed

            if (gamepad2.cross){
                shooterState = true;
            }

            if (gamepad2.circle){
                shooterState = false;
            }

            if (shooterState){
                stV =  shooter.calculate(1500, shooter.getVelocity());
            } else {
                stV = 0;
            }

            // Run the hood
//            if(gamepad2.cross) {
//                hdV = HOOD;
//                // Manual mode
//                if(gamepad2.left_bumper) {
//                    hdV = -hdV;
//                }
//                // Auto mode
//                else if(gamepad2.right_bumper) {
//                    hdAutoMode = true;
//                }
//            }

            if(gamepad1.left_bumper) {
                ikV = -ikV;
            }

            // While the devices are working, Triggering the L2 of gamepad 2 will reverse the direction of all of them. (in their manual mode)
            if(gamepad2.left_trigger > 0.8) {
                stV = -stV;
                clbV = -clbV;
            }
        }

        if (gateButton && !oldGatePosition){
            if (gatePosition == 0){
                loaderGate.OpenGate(0.7);
                gatePosition = 1;
            } else {
                loaderGate.OpenGate(0);
                gatePosition = 0;
            }
        }

        oldGatePosition = gateButton;


        if (grabButton && !oldGrabPosition){
            if (grabPosition == 0){
                grab.grabPos(0.7);
                grabPosition = 1;
            } else {
                grab.grabPos(0);
                grabPosition = 0;
            }
        }

        oldGrabPosition = grabButton;

        if (gamepad2.dpad_up){
            ldV = LOADER;
        } else if (gamepad1.dpad_down){
            ldV = -LOADER;
        }


        intake.setMotorPower(ikV);
        loader.load(ldV);
        drivebase.setMotorPower(rightDrB, leftDrB);
        shooter.shoot(stV);

        webcam.cameraStream();

        telemetry.addData("Test mode", testMode);
        telemetry.addData("Shooter velocity", shooter.getVelocity());
        telemetry.addData("Shooter Power", shooter.getMotorPower());
        telemetry.addData("Intake speed", ikV);
        telemetry.addData("Loader state", ldV);
        telemetry.addData("left DrB speed", leftDrB);
        telemetry.addData("Right DrB speed", rightDrB);
        telemetry.addData("Max DrB speed", MAX_SPEED);
        telemetry.addData("Grab Pos", grabPosition);
        telemetry.addData("Gate Pos", gatePosition);
//        telemetry.addData("Climber speed", clbV);
        telemetry.update();
    }
}
