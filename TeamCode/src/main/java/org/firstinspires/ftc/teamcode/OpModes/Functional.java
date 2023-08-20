package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;
import org.firstinspires.ftc.teamcode.VNRobot.FunctionalRobot;

@TeleOp(name = "Functional")
@Disabled
public class Functional extends OpMode {
    private FunctionalRobot robot;



    @Override
    public void init(){
        robot = new FunctionalRobot(this);
        robot.init();

    }
//
    @Override
    public void loop(){
//

        robot.loop();
    }
}
