package org.firstinspires.ftc.teamcode.Subsystems;



import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;



//@TeleOp (name="SixWheel")
public class Drivebase {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    BNO055IMU imu;
    private HardwareMap hardwareMap;

    public Drivebase(OpMode opMode){
        this.hardwareMap = opMode.hardwareMap;
    }


    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class,"rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

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

    public void setAllMotorPower(double p) {setMotorPower(p,p);}
}