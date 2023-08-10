package org.firstinspires.ftc.teamcode.VNRobot;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Camera360;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Grab;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalServo;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.VerticalServo;

public class FunctionalRobot {

//    private IMU imu;
    private Drivebase drivebase;
    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
//    private Intake intake;
//    private Shooter shooter;

    private HorizontalServo horizontalServo;
    private VerticalServo verticalServo;

    private Grab grabLeft;
    private Grab grabRight;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    private  boolean shooterState = false;

    public FunctionalRobot (OpMode opMode){
//        imu = new IMU(opMode);
        drivebase = new Drivebase(opMode);
//        intake = new Intake(opMode);
//        shooter = new Shooter(opMode);

        horizontalServo = new HorizontalServo(opMode);
        verticalServo = new VerticalServo(opMode);
        grabLeft = new Grab(opMode);
        grabRight = new Grab(opMode);

        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;


    }
   public void init(){
//        imu.init();
        drivebase.init();
//        intake.init();
//        shooter.init();
        horizontalServo.init();
        verticalServo.init();
        grabLeft.init();
        grabRight.init();
    }

    public void loop(){
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
    }
    public void runOpMode(){
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        boolean reverseState = false;
        boolean intakeToggle = false;
        double intakePower = 0;


        // Camera 360o

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            verticalServo.setVerticalPos(0.1);
        } else if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down){
            verticalServo.setVerticalPos(-0.1);
        }

        if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            horizontalServo.setHorizontalPos(0.1);
        } else if (!currentGamepad1.dpad_left && previousGamepad1.dpad_left){
            horizontalServo.setHorizontalPos(-0.1);
        }

        // Reverse drivebase

        if(currentGamepad1.touchpad && !previousGamepad1.touchpad) {
            reverseState = !reverseState;
        }

        if(reverseState){
            drivebase.setMotorPower(-left,-right);
        } else {
            drivebase.setMotorPower(left,right);
        }


        if (currentGamepad1.triangle && !previousGamepad1.triangle){
            intakeToggle = !intakeToggle;
        }
        if(intakeToggle){
            intakePower = 1;
        } else {
            intakePower = 0;
        }

        if (currentGamepad1.cross && !previousGamepad1.cross){
            intakeToggle = !intakeToggle;
        }
        if(intakeToggle){
            intakePower = -1;
        } else {
            intakePower = 0;
        }

        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
            grabLeft.grabPos(0.2);
            grabRight.grabPos(0.2);
        } else if (!currentGamepad1.right_bumper && previousGamepad1.right_bumper){
            grabLeft.grabPos(-0.2);
            grabRight.grabPos(-0.2);
        }


//        if (gamepad1.triangle){
//            intakePower = 1;
//        } else if (gamepad1.cross){
//            intakePower = -1;
//        } else {
//            intakePower = 0;
//        }


        if (gamepad1.square){
            shooterState = true;
        }

        if (gamepad1.circle){
            shooterState = false;
        }

        drivebase.setMotorPower(left,right);
//        intake.setMotorPower(intakePower);

        telemetry.addData("Shooter is calibrating", shooterState);
        telemetry.addData("Left Power", left);
        telemetry.addData("Right Power", right);
        telemetry.update();
    }



}
