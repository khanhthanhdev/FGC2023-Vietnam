package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "Velocity Max")
public class FindVelocity extends LinearOpMode {

    private DcMotorEx leftMotor;

    private double maxPower = 1.0;
    private double zeroPower = 0.0;
    private double currentVelocity = 0.0;
    private double maxVelocity = 0.0;

    private double shooterTargetVelocity = 1500.0;
    private double shooterMaxVelocity = 2940.0;
    private double F = 32767.0 / shooterMaxVelocity;
    private double kP = 1.3;
    private double kI = kP * 0.1;
    private double kD = kP * 0.01;
    private double position = 5.0;

    @Override
    public void runOpMode(){
        initHardware();
        while (!isStarted()){
            motorTelemetry();
        }
        waitForStart();
        while (opModeIsActive()){

            if (gamepad2.cross){
                runShooter(shooterTargetVelocity);
            } else if (gamepad2.circle) {
                leftMotor.setPower(zeroPower);
            }

            motorTelemetry();
        }
    }

    private void initHardware(){
        initShooterPID(kP, kI,kD,F,position);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void control(){
        if(gamepad1.circle){
            leftMotor.setPower(maxPower);
        }
        if(gamepad1.cross){
            leftMotor.setPower(zeroPower);
        }
    }

    public void initShooterPID(double kP, double kI, double kD, double F, double position){
        leftMotor = hardwareMap.get(DcMotorEx.class, "shooter");
        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftMotor.setPower(zeroPower);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setVelocityPIDFCoefficients(kP,kI,kD,F);
        leftMotor.setPositionPIDFCoefficients(position);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runShooter(double velocity){
        leftMotor.setVelocity(velocity);
        currentVelocity = leftMotor.getVelocity();
        if (currentVelocity > maxVelocity){
            maxVelocity = currentVelocity;
        }
    }


    public void motorTelemetry(){
//        telemetry.addData("Current Power", leftMotor.getPower());
//        telemetry.addData("Max Velocity", maxVelocity);
//        telemetry.addData("Current Velociry", currentVelocity);
        telemetry.addData("Power", leftMotor.getPower());
        telemetry.addData("Target Velocity", shooterTargetVelocity);
        telemetry.addData("Current Velocity", leftMotor.getVelocity());
        telemetry.addData("F",F);
        telemetry.addData("kP",kP);
        telemetry.addData("kI",kI);
        telemetry.addData("kD",kD);
        telemetry.update();
    }
}
