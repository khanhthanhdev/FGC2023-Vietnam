package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class WrapBall {
    private CRServo wrap;
    private final HardwareMap hardwareMap;
    public WrapBall(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    // Initialize the loader
    public void init() {
        wrap = hardwareMap.get(CRServo.class, "wrap");
        wrap.setDirection(DcMotorSimple.Direction.FORWARD);
        wrap.resetDeviceConfigurationForOpMode();
    }

    // Activate the loader
    public void wrapSpped(double speed) {
        wrap.setPower(speed);
    }
}
