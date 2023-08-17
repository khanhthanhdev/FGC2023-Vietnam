package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Config
@TeleOp (name = "Dashboard test")

public class DashboardFTC extends OpMode {

    private PIDFController controller;

    public static double p =0,i=0,d=0;

    public static double f = 0;
    public static int target = 0;

    private DcMotorEx shooter;

    @Override
    public void init(){
        controller = new PIDFController(p,i,d,f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void loop(){

        controller.setPIDF(p,i,d, f);
        int shooterPos = shooter.getCurrentPosition();

        double pid = controller.calculate(target, shooterPos);

        double power = pid;

        shooter.setPower(power);

        telemetry.addData("ArmPos", shooterPos);
        telemetry.addData("Target", target);
        telemetry.addData("power", power);
        telemetry.update();

    }
}
