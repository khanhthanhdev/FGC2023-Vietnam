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
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void shoot(double angularRate) {
        shooter.setVelocity(angularRate, AngleUnit.RADIANS);
    }

    public double getVelocity() {
        return shooter.getVelocity(AngleUnit.RADIANS);
    }

}