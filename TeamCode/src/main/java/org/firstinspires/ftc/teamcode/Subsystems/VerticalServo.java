package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class VerticalServo {

    private Servo vertical;

    private HardwareMap hardwareMap;

    public VerticalServo(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        vertical = hardwareMap.get(Servo.class, "vertical");

    }

    public void setVerticalPos(double pos){vertical.setPosition(vertical.getPosition()+pos);}
}
