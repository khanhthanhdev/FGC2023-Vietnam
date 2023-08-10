package org.firstinspires.ftc.teamcode.VNRobot;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Camera 360")
public class ServoCamera extends OpMode {
    private Servo horizontal;
    private Servo vertical;
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init(){
        horizontal = hardwareMap.get(Servo.class, "horizontal");
        vertical = hardwareMap.get(Servo.class, "vertical");
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop(){
        previousGamepad1.copy(currentGamepad1);
        currentGamepad1.copy(gamepad1);

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
    }
}
