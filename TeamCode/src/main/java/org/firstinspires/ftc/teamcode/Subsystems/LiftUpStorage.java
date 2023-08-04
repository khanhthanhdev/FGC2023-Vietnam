package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LiftUpStorage {
    private DcMotorEx cascadeLeft;
    private DcMotorEx cascadeRight;

    private HardwareMap hardwareMap;

    public LiftUpStorage(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
    }

    public void init(){
        cascadeLeft = hardwareMap.get(DcMotorEx.class, "cascadeLeft");
        cascadeRight = hardwareMap.get(DcMotorEx.class, "cascadeRight");

        cascadeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cascadeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void liftUp(double speed){
        cascadeLeft.setPower(speed);
        cascadeRight.setPower(speed);
    }
}
