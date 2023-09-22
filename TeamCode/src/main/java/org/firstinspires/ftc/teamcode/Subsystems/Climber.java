package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for controlling the climber, with 1 motors
public class Climber {
    private CRServo clb;
    private HardwareMap hardwareMap;
    public Climber(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        clb = hardwareMap.get(CRServo.class, "climb");

        clb.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        // Set the Brake mode for the motor


    }

    // Function for climbing
    public void climb(double speed) {
        clb.set(speed);

    }

    public void revertClimb(){
        clb.getInverted();
    }
}