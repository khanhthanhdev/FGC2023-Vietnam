package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Grab {

    private CRServo grabLeft;
    private CRServo grabRight;

    private final HardwareMap hardwareMap;

    public Grab(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        grabLeft = hardwareMap.get(CRServo.class, "grabLeft");
        grabRight = hardwareMap.get(CRServo.class, "grabRight");
    }

    public void grabSpeed(double speed){
        grabLeft.setPower(speed);
        grabRight.setPower(-speed);
    }
}
