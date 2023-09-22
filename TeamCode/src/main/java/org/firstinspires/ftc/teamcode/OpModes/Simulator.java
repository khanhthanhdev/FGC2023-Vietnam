package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

public class Simulator extends LinearOpMode {

    private DcMotor frontRight;
    private DcMotor backRight;
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor motor5;
    private DcMotor motor6;
    private DcMotor motor7;
    private DcMotor motor8;


    @Override
    public void runOpMode(){
        double y;
        double x;
        float rx;
        double denominator;
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        motor5 = hardwareMap.get(DcMotor.class, "motor5");


        // Reverse the right side motors.  This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards, reverse the left side instead.
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            // Remember, Y stick value is reversed
            y = -gamepad1.left_stick_y;
            // Factor to counteract imperfect strafing
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1.
            // This ensures all powers maintain the same ratio, but only if one is outside of the range [-1, 1].
            denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
            // Make sure your ID's match your configuration
            frontLeft.setPower((y + x + rx) / denominator);
            backLeft.setPower(((y - x) + rx) / denominator);
            frontRight.setPower(((y - x) - rx) / denominator);
            backRight.setPower(((y + x) - rx) / denominator);
            // to use grabber to pick up atom
            if (gamepad1.left_bumper) {
                motor5.setPower(1);
            } else if (gamepad1.right_bumper) {
                // motor 5 (-1) release  the  atom
                motor5.setPower(-1);
            } else {
                // when power is "0"  nothing happens
                motor5.setPower(0);
            }

            if (gamepad1.circle){
                motor7.setPower(1);
            } else if (gamepad1.cross){
                motor7.setPower(-1);
            } else {
                motor7.setPower(0);
            }

            if (gamepad1.square){
                motor6.setPower(1);
            } else if (gamepad1.triangle){
                motor6.setPower(-1);
            } else {
                motor6.setPower(0);
            }

            if (gamepad1.dpad_up){
                motor8.setPower(1);
            } else if (gamepad1.dpad_down){
                motor8.setPower(-1);
            } else {
                motor8.setPower(0);
            }

        }
    }
}
