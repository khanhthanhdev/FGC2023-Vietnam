package org.firstinspires.ftc.teamcode.OpModes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VNRobot.OneGamepad;
@TeleOp(name = "OneGamepad")
public class OneHandDrive extends OpMode {
    private OneGamepad robot;

    @Override
    public void init() {
        robot = new OneGamepad(this);
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
