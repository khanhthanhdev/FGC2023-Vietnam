package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Camera360 {

    private Servo horizontal;
    private Servo vertical;

    private HardwareMap hardwareMap;

    public Camera360(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        horizontal = hardwareMap.get(Servo.class, "horizontal");

        vertical = hardwareMap.get(Servo.class, "vertical");
    }
}
