package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Test")
public class Test extends OpMode {
    private DcMotor testMotor;

    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotor.class, "test");
        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        double speed = gamepad1.left_stick_y;
        if(gamepad1.circle) {
            testMotor.setPower(speed);
        }
        telemetry.addData("Speed", speed);
        telemetry.update();
    }
}