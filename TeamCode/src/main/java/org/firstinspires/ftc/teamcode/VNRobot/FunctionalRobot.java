package org.firstinspires.ftc.teamcode.VNRobot;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.Grab;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LoaderGate;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class FunctionalRobot {

//    private IMU imu;
    private final Drivebase drivebase;
    private final Telemetry telemetry;
    private final Gamepad gamepad1;
    private final Intake intake;
    private final Shooter shooter;

    private final LoaderGate loaderGate;

    private final Grab grabLeft;
    private final Grab grabRight;

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    boolean shooterState = false;
    boolean reverseState = false;
    boolean intakeToggle = false;
    double gatePosition;
    boolean oldGatePosition;

    double grabPosition;

    boolean oldGrabPosition;



    public FunctionalRobot (OpMode opMode){
//        imu = new IMU(opMode);
        drivebase = new Drivebase(opMode);
        intake = new Intake(opMode);
        shooter = new Shooter(opMode);

        loaderGate = new LoaderGate(opMode);
        grabLeft = new Grab(opMode);
        grabRight = new Grab(opMode);

        telemetry = opMode.telemetry;
        gamepad1 = opMode.gamepad1;
        gamepad2 = opMode.gamepad2;


    }
   public void init(){
//        imu.init();
        drivebase.init();
        intake.init();
        shooter.init();
        loaderGate.init();
        grabLeft.init();
        grabRight.init();
    }

    public void loop(){
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;
        boolean gateButton = gamepad2.cross;
        boolean grabButton = gamepad1.dpad_down;
        double intakePower = 0;



        // Reverse drivebase

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
                grabLeft.grabPos(0.5);
                grabRight.grabPos(0.5);
                gatePosition = 1;
            } else {
                grabLeft.grabPos(0);
                grabRight.grabPos(0);
                gatePosition = 0;
            }
        }

        oldGrabPosition = grabButton;



        if (gamepad1.left_stick_y > 0.6 || gamepad1.right_stick_y > 0.6){
            if (gamepad1.left_stick_y >0.6){
                drivebase.setMotorPower(0.6,right);
            }
        } else if (gamepad1.left_stick_y < -0.6 && gamepad1.right_stick_y < -0.6){
            drivebase.setMotorPower(-0.6, -0.6);
        } else {
            drivebase.setAllMotorPower(0);
        }


        boolean intakeState = false;
        if (gamepad1.left_bumper){
            if (intake.getMotorPower() == 0){
                intake.setMotorPower(-1);
                intakeState = true;
            } else {
                intake.setMotorPower(0);
                intakeState = false;
            }
        } else if (gamepad1.right_bumper){
            if (intake.getMotorPower() == 0){
                intake.setMotorPower(1);
            } else {
                intake.setMotorPower(0);
                intakeState = false;
            }
        }


        if (gamepad2.square){
            shooterState = true;
        }

        if (gamepad2.circle){
            shooterState = false;
        }

        if (shooterState){
            shooter.shoot(-0.7);
        } else {
            shooter.shoot(0);
        }


        intake.setMotorPower(intakePower);

        telemetry.addData("Shooter is calibrating", shooterState);
        telemetry.addData("Shooter Power", shooter.getVelocity());
        telemetry.addData("Gate Pos", gatePosition);
        telemetry.addData("Intake State", intakeToggle);
        telemetry.addData("Left Power", left);
        telemetry.addData("Right Power", right);
        telemetry.update();
    }

}
