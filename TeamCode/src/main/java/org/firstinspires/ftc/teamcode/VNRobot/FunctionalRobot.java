package org.firstinspires.ftc.teamcode.VNRobot;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

public class FunctionalRobot {

//    private IMU imu;
    private Drivebase drivebase;
    private Telemetry telemetry;
    private Gamepad gamepad;
//    private Intake intake;
//    private Shooter shooter;
    private  boolean shooterState = false;

    public FunctionalRobot (OpMode opMode){
//        imu = new IMU(opMode);
        drivebase = new Drivebase(opMode);
//        intake = new Intake(opMode);
//        shooter = new Shooter(opMode);

        telemetry = opMode.telemetry;
        gamepad = opMode.gamepad1;
    }

    public void init(){
//        imu.init();
        drivebase.init();
//        intake.init();
//        shooter.init();
    }

    public void runOpMode(){
        double left = gamepad.left_stick_y;
        double right = gamepad.right_stick_y;
        double intakePower = 0;
        boolean TouchpadPressed = false;
        boolean driveOn = false;

        if (gamepad.touchpad){
            driveOn = !driveOn;
            telemetry.addData("Drive Reverse", driveOn);

            if (driveOn){
                left = gamepad.left_stick_y;
                right = gamepad.right_stick_y;
            }
            else {
                left = -gamepad.left_stick_y;;
                right = -gamepad.right_stick_y;
            }
        }


//        if (gamepad.left_bumper){
//                left = left;
//                right = right;
//
//        }else if (gamepad.right_bumper){
//                left = -left;
//                right = -right;
//        }

        GamepadEx gamepadEx = new GamepadEx(gamepad);

        ButtonReader reverse = new ButtonReader(
                gamepadEx, GamepadKeys.Button.DPAD_UP
        );

        reverse.wasJustPressed();


        if (gamepad.triangle){
            intakePower = 1;
        } else if (gamepad.cross){
            intakePower = -1;
        } else {
            intakePower = 0;
        }


        if (gamepad.square){
            shooterState = true;
        }

        if (gamepad.circle){
            shooterState = false;
        }


        drivebase.setMotorPower(left,right);
//        intake.setMotorPower(intakePower);

        telemetry.addData("Shooter is calibrating", shooterState);
        telemetry.update();
    }


}
