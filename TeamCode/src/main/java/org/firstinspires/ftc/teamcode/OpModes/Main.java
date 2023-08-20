package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VNRobot.Robot;

@TeleOp(name = "Main")
public class Main extends OpMode {
    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public  void start(){
        robot.start();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}