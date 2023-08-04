package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VNRobot.FunctionalRobot;

@TeleOp(name = "Functional")
public class Functional extends OpMode {
    private FunctionalRobot robot;

    @Override
    public void init(){
        robot = new FunctionalRobot(this);
        robot.init();
    }

    @Override
    public void loop(){
        robot.runOpMode();
    }
}
