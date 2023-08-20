package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class LoaderGate {

    private Servo loaderGate;

    private final HardwareMap hardwareMap;

    public LoaderGate(OpMode opMode){hardwareMap = opMode.hardwareMap;}

    public void init(){
        loaderGate = hardwareMap.get(Servo.class, "loaderGate");

    }

    public void OpenGate(double pos){loaderGate.setPosition(pos);}
}
