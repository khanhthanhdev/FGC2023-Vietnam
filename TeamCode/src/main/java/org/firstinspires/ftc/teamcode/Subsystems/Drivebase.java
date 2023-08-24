package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.Constants.ROTATION.*;


//@TeleOp (name="SixWheel")
public class Drivebase {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    BNO055IMU imu;
    private final HardwareMap hardwareMap;
    private final PIDController controller;
    public Drivebase(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.controller = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    }

    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

//        controller.setTolerance(TOLERANCE);
//        controller.setIntegrationBounds(INTERGRAL_MIN, INTERGRAL_MAX);
    }

    public void setMotorPower(double left, double right){
        double largest = 1.0;

        largest = Math.max(largest, Math.abs(left));
        largest = Math.max(largest, Math.abs(right));

        leftFront.setPower(left/largest);
        leftBack.setPower(left/largest);
        rightFront.setPower(right/largest);
        rightBack.setPower(right/largest);

    }

    // Return the value of encoder of both sides
    public int getLeftPosition() {
        return leftFront.getCurrentPosition();
    }

    public int getRightPosition() {
        return rightFront.getCurrentPosition();
    }

//    public double rotateAngle(double angle, double setPoint) {
//        controller.setSetPoint(setPoint);
//        return controller.calculate(angle);
//    }
//
//    // Check whether our robot has faced the sink or not.
//    public boolean atSetpoint() {
//        return controller.atSetPoint();
//    }

    public void setAllMotorPower(double p) {setMotorPower(p,p);}
}