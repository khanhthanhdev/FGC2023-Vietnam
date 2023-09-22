package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftUp {
    private DcMotor lift;
    private final HardwareMap hardwareMap;

    public LiftUp(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void Lift(double speed){
        lift.setPower(speed);
    }

}
