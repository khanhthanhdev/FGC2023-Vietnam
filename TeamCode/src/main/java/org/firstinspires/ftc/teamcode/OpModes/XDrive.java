package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "XDriveBot demo", group = "XBot")

public class XDrive extends LinearOpMode {

    DcMotor m1, m2, m3, m4;
//    CRServo backServo;

    public void runOpMode(){

        m1 = hardwareMap.dcMotor.get("leftFront");
        m2 = hardwareMap.dcMotor.get("rightFront");
        m3 = hardwareMap.dcMotor.get("leftBack");
        m4 = hardwareMap.dcMotor.get("rightBack");

        m4.setDirection(DcMotor.Direction.REVERSE);
        m2.setDirection(DcMotor.Direction.REVERSE);
        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = false;
        parameters.loggingTag = "Who cares.";

        imu.initialize(parameters);

        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            double vertical = -gamepad1.left_stick_y;
            double horizontal = gamepad1.left_stick_x;
            double pivot = gamepad1.right_stick_x;
            m1.setPower(pivot+vertical+horizontal);
            m2.setPower(-pivot+vertical-horizontal);
            m3.setPower(pivot+vertical-horizontal);
            m4.setPower(-pivot+vertical+horizontal);

            Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            telemetry.addData("Heading", " %.1f", orientation.firstAngle * 180.0 / Math.PI);

            telemetry.addData("Encoders"," %d %d %d %d", m1.getCurrentPosition(), m2.getCurrentPosition(),
                    m3.getCurrentPosition(), m4.getCurrentPosition());
            telemetry.update();
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}