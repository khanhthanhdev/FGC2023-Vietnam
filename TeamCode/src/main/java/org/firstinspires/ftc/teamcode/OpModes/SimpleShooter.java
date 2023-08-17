package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp (name="Simple Shooter PID")
public class SimpleShooter extends LinearOpMode {

    DcMotorEx shooter;
    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 1;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            double shooterPower = PIDControl(1000, shooter.getVelocity());
            shooter.setPower(shooterPower);
        }
    }
    public double PIDControl(double reference, double state){
        double error = reference - state;

        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        telemetry.addData("Pos", state);
        telemetry.addData("Target", reference);
        telemetry.addData("Error", error);
        telemetry.update();
        return output;
    }

}