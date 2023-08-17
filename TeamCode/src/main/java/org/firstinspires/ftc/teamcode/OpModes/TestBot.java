package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalServo;


@TeleOp(name="Test Robot 1")
public class TestBot  extends OpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotorEx shooter;

    private Servo grabLeft;
    private Servo grabRight;

    private double servoPosition;
    private boolean oldServoPosition;




    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
//
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void loop(){
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();


        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        boolean reverseState = false;
        boolean shooterToggle = false;
        boolean intakeToggle = false;
        boolean grabButton = gamepad1.circle;

        if(currentGamepad1.touchpad && !previousGamepad1.touchpad) {
            reverseState = !reverseState;
        }

        if(reverseState){
            setMotorPower(-right,-left);
        } else {
            setMotorPower(left,right);
        }

        if (currentGamepad1.square && !previousGamepad1.square){
            shooterToggle = !shooterToggle;
        }
        if(shooterToggle){
            shooter.setPower(PIDControl(1000, shooter.getVelocity()));
        } else {
            shooter.setPower(0);
        }


        if (currentGamepad1.cross && !previousGamepad1.cross) {
            intakeToggle = !intakeToggle;
        } else if (!currentGamepad1.cross && previousGamepad1.cross) {
            intakeToggle = !intakeToggle;
        }
        if (intakeToggle) {
            intake.setPower(1);
        }
        else {
            intake.setPower(0);
        }


        if (grabButton && !oldServoPosition) {
            if (servoPosition == 0){
                grabLeft.setPosition(0.75);
                grabRight.setPosition(0.75);
                servoPosition= 0.75;
            } else {
                grabLeft.setPosition(0);

                grabRight.setPosition(0);
                servoPosition = 0;
            }
        }

        oldServoPosition = grabButton;


        telemetry.addData("grabRight ", grabRight.getPosition());
        telemetry.addData("grabLeft ", grabLeft.getPosition());
        telemetry.addData("Intake state", intakeToggle);
        telemetry.update();


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

    public double PIDControl(double reference, double state){
        double error = reference - state;
        double integralSum = 0;
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double Kf = 10;

        ElapsedTime timer = new ElapsedTime();
        double lastError = 0;

        integralSum += error + timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        lastError = error;


        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }

    public void GrabLeftPos(double pos){
        grabLeft.setPosition(grabLeft.getPosition()+pos);
    }
    public void GrabRightPos(double pos){
        grabRight.setPosition(grabRight.getPosition()+pos);
    }

}
