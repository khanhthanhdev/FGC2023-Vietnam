package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private DcMotor intake;

    private HardwareMap hardwareMap;

    public Intake(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
    }

    public void init(){
        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setMotorPower(double speed){
        intake.setPower(speed);
    }

    public double getMotorPower(){return intake.getPower();}
}
