package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for controlling the climber, with 1 motors
public class Climber {
    private DcMotor clb1;
    private HardwareMap hardwareMap;
    public Climber(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        clb1 = hardwareMap.get(DcMotor.class, "climber");

        // Each motor will operate in the opposite direction eith the other one,
        // so that the mechanism will move in a single direction
        clb1.setDirection(DcMotorSimple.Direction.FORWARD);
        // Set the Brake mode for the motor
        clb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    // Function for climbing
    public void climb(double speed) {
        clb1.setPower(speed);

    }
}