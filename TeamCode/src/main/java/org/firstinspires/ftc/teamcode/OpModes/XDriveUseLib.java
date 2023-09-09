package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "FTCLIB drivebase")
@Disabled
public class XDriveUseLib extends LinearOpMode {



    @Override
    public void runOpMode(){
        boolean FIELD_CENTRIC = false;
        HDrive drive = new HDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack")
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();


        while (!isStopRequested()){

            if (!FIELD_CENTRIC){
                drive.driveRobotCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX()
                );
            } else {
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX(),
                        imu.getRotation2d().getDegrees()   // gyro value passed in here must be in degrees
                );
            }

        }
    }
}
