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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

    }

    public void setMotorPower(double left, double right){
        leftFront.setPower(left);
        leftBack.setPower(left);
        rightFront.setPower(right);
        rightBack.setPower(right);

    }

    public void setAllMotorPower(double p) {setMotorPower(p,p);}
}