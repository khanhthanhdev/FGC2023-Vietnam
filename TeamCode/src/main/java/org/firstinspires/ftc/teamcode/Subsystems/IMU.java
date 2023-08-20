package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

// The IMU, to return the headings of robot, use for robot odometry.
public class IMU {
    private BNO055IMU imu;
    private final HardwareMap hardwareMap;

    public IMU(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    // Initialize the IMU
    public void init() {
        imu = hardwareMap.get(BNO055IMU.class, "IMU");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; // We use radians as our basic units
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; // We use m/s^2 as the unit for acceleration

        imu.initialize(parameters);
    }

    // Return the yaw angle of robot
    public double getYaw() {
        return imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;
    }
}