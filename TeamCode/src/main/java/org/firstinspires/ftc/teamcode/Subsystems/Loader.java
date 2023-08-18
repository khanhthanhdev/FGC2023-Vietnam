package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Class for controlling the loader
public class Loader {
    private CRServo loader;
    private HardwareMap hardwareMap;
    public Loader(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    // Initialize the loader
    public void init() {
        loader = hardwareMap.get(CRServo.class, "loader");
        loader.setDirection(DcMotorSimple.Direction.FORWARD);
        loader.resetDeviceConfigurationForOpMode();
    }

    // Activate the loader
    public void load(double speed) {
        loader.setPower(speed);
    }
}