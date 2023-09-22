package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Grab {

    private CRServo grab;
    private AnalogInput potentiometer;

    private final HardwareMap hardwareMap;
    private Telemetry telemetry;
    double currentVoltage;
    double outputAngle;

    public Grab(OpMode opMode){
        hardwareMap = opMode.hardwareMap;
        telemetry = opMode.telemetry;
    }

    public void init(){
        grab = hardwareMap.get(CRServo.class, "grab");
        potentiometer = hardwareMap.get(AnalogInput.class, "Potentiometer");
    }

    public void grabSpeed(double speed){
        grab.setPower(speed);

    }
    public double getVol(){
        return currentVoltage = potentiometer.getVoltage();

    }
    public double getAngle(){
        currentVoltage = potentiometer.getVoltage();
        return outputAngle = (445.5*(currentVoltage-270)) / (currentVoltage*currentVoltage - 270*currentVoltage - 36450);
    }
}
