package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

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
    @Override
    public void init(){
        robot = new FunctionalRobot(this);
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        vertical = hardwareMap.get(Servo.class, "vertical");
        robot.init();

    }

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

        robot.runOpMode();
    }
}
