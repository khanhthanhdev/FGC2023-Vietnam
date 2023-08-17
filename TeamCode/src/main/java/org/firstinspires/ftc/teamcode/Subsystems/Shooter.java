package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    private DcMotorEx shooter;
    private HardwareMap hardwareMap;

    public Shooter(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shoot(double power) {
        shooter.setPower(power);
    }

    public double getMotorPower(){
        return shooter.getPower();
    }

}