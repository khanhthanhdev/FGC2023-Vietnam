package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Grab {

    private Servo grabLeft;
    private Servo grabRight;

    private HardwareMap hardwareMap;

    public Grab(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        grabLeft = hardwareMap.get(Servo.class, "grabLeft");
        grabRight = hardwareMap.get(Servo.class, "grabRight");
    }

    public void grabPos(double pos){
        grabLeft.setPosition(pos);
        grabRight.setPosition(pos);
    }
}
