package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HDrive drivebase")
@Disabled
public class HDriveLib extends LinearOpMode {



    @Override
    public void runOpMode(){
        boolean FIELD_CENTRIC = false;
        HDrive drive = new HDrive(
                new Motor(hardwareMap, "leftFront"),
                new Motor(hardwareMap, "rightFront"),
                new Motor(hardwareMap, "leftBack"),
                new Motor(hardwareMap, "rightBack"),
                new Motor(hardwareMap,"center")
        );

        RevIMU imu = new RevIMU(hardwareMap);
        imu.init();

        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();


        while (!isStopRequested()){

            if (gamepad1.cross){
                FIELD_CENTRIC = true;
            } if (gamepad1.circle){
                FIELD_CENTRIC = false;
            }

            if (FIELD_CENTRIC == true){
                drive.driveRobotCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        driverOp.getRightX()
                );
            } else if (FIELD_CENTRIC == false){
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
