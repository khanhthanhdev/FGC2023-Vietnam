package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp (name="HDrive Strafer")
public class HDrive extends OpMode {

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor center;
    private DcMotor cascade;

    public void setMotorPower(double leftPower, double rightPower){
        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightFront.setPower(rightPower);
        rightBack.setPower(rightPower);
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

        double speed = -gamepad1.left_stick_y;
        double strafer = gamepad1.right_stick_x;


        double leftPower = Range.clip(speed+strafer,-1.0,1.0);
        double rightPower = Range.clip(speed-strafer, -1.0,1.0);

        center.setPower(gamepad1.left_stick_x);

        setMotorPower(leftPower, rightPower);

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