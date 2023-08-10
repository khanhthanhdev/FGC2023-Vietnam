package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HorizontalServo {

    private Servo horizontal;

    private HardwareMap hardwareMap;

    public HorizontalServo(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        horizontal = hardwareMap.get(Servo.class, "horizontal");

    }

    public void setHorizontalPos(double pos){horizontal.setPosition(horizontal.getPosition()+pos);}
}
