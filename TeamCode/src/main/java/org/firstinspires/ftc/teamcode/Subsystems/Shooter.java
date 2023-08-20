package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.firstinspires.ftc.teamcode.Constants.SHOOT.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// Our class to control the shooter. Our shooter using 2 dc motor and a pid controller to reach a certain velocity.
public class Shooter {
    private DcMotorEx sr1;
    private final PIDController controller;
    private final HardwareMap hardwareMap;

    double p = 1.3;
    double i = 0;
    double d = 0;

    private Telemetry telemetry;

    public Shooter(OpMode opMode) {
        controller = new PIDController(p, i,d);
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;

    }

    public void init() {
        sr1 = hardwareMap.get(DcMotorEx.class, "shooter");
        sr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sr1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sr1.setDirection(DcMotorSimple.Direction.REVERSE);

        // PID controller for shooting
        controller.setTolerance(POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        controller.setIntegrationBounds(MIN_INTERGRAL, MAX_INTERGRAL);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    // Both motors move with the same speed and direction.
    public void shoot(double velocity) {
        sr1.setVelocity(velocity);
    }

    // Return the angle rates of the motor using encoder.
    public double getVelocity() {
        return sr1.getVelocity(AngleUnit.RADIANS);
    }

    public double getMotorPower(){
        return sr1.getPower();
    }

    // Use the pid controller to allow the shooter to reach the needed velocity with high accuracy
    public double calculate(double position, double measurement) {
        telemetry.addData("Measurement", measurement);
        telemetry.addData("Target", position);
        telemetry.update();
        return controller.calculate(measurement, position);
    }


}