package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.VNRobot.FunctionalRobot;

@TeleOp(name = "Functional")
public class Functional extends OpMode {
    private FunctionalRobot robot;
    private Servo horizontal;
    private Servo vertical;

    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    boolean reverseState = false;
    boolean intakeToggle = false;
    double intakePower = 0;


    @Override
    public void init(){
        robot = new FunctionalRobot(this);
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        vertical = hardwareMap.get(Servo.class, "vertical");
        robot.init();

    }
//
    @Override
    public void loop(){
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);


        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            vertical.setPosition(vertical.getPosition()+0.1);
        } else if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down){
            vertical.setPosition(vertical.getPosition()-0.1);
        }

        if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right){
            horizontal.setPosition(horizontal.getPosition()+0.1);
        } else if (!currentGamepad1.dpad_left && previousGamepad1.dpad_left){
            horizontal.setPosition(horizontal.getPosition()-0.1);
        }

        if(currentGamepad1.touchpad && !previousGamepad1.touchpad) {
            reverseState = !reverseState;
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

        robot.runOpMode();
    }
}
