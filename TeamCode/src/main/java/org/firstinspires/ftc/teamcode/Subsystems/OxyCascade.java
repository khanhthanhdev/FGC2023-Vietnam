package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OxyCascade {
    private CRServo OxyCascade;
    private HardwareMap hardwareMap;
    public OxyCascade(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    // Initialize the loader
    public void init() {
        OxyCascade = hardwareMap.get(CRServo.class, "oxi");
        OxyCascade.setDirection(DcMotorSimple.Direction.FORWARD);
        OxyCascade.resetDeviceConfigurationForOpMode();
    }

    // Activate the loader
    public void OxyLiftUp(double speed) {
        OxyCascade.setPower(speed);
    }
}
