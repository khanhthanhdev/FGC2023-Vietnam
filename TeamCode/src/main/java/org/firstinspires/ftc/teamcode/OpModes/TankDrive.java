package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class TankDrive extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor center;

    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        center = hardwareMap.get(DcMotor.class,"center");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        center.setDirection(DcMotorSimple.Direction.FORWARD);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        center.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    @Override
    public void loop(){
        double MAX_SPEED = 0.6;
        double leftDrB = 0;
        double rightDrB = 0;

        if (gamepad1.left_trigger > 0.5) {
            MAX_SPEED = 1.0;
        }

        leftDrB = gamepad1.left_stick_y * MAX_SPEED;
        rightDrB = gamepad1.right_stick_y * MAX_SPEED;

        setMotorPower(leftDrB,rightDrB);

    }
}
