package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Constants.SPEED.BOOST_DrB;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp (name="HDrive")
public class HDrive extends OpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor center;
    private DcMotor cascade;
    public void goStop() {
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        center.setPower(0);
    }

    public void setMotorPower(double speed){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }


    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        center = hardwareMap.get(DcMotor.class, "center");
        cascade  =hardwareMap.get(DcMotor.class, "cascade");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


    }



    @Override
    public void loop() {


        double NORMAL_SP = 0.6;
        double BOOST_SP = 1;
        double speed = 0;
        double strafer = 0;

        if (gamepad1.left_trigger > 0.5) {
            NORMAL_SP = BOOST_SP;
        }

        speed = gamepad1.left_stick_y * NORMAL_SP;
        strafer = gamepad1.right_stick_x * NORMAL_SP;

//        if (gamepad1.left_stick_y > 0.05){
//            left.setPower(gamepad1.left_stick_y);
//        } else if (gamepad1.left_stick_y < -0.05){
//            left.setPower(gamepad1.left_stick_y);
//        } else if (gamepad1.left_stick_y <= 0.05 || gamepad1.left_stick_y >= -0.05){
//            left.setPower(0);
//        }
//
//        if (gamepad1.right_stick_y > 0.05){
//            right.setPower(gamepad1.right_stick_y);
//        } else if (gamepad1.right_stick_y < -0.05){
//            right.setPower(gamepad1.right_stick_y);
//        } else if (gamepad1.right_stick_y <= 0.1|| gamepad1.right_stick_y >= -0.1){
//            right.setPower(0);
//        }

        if (gamepad1.right_stick_x > 0.05){
            center.setPower(strafer);
        } else if (gamepad1.right_stick_x < -0.05){
            center.setPower(strafer);
        } else if (gamepad1.right_stick_x <= 0.05 || gamepad1.right_stick_x >= -0.05) {
            center.setPower(0);
        }
        setMotorPower(speed);

        if(gamepad1.right_bumper){
            cascade.setPower(1);
        }
        else if (gamepad1.left_bumper){
            cascade.setPower(-1);
        } else {
            cascade.setPower(0);
        }

    }
}