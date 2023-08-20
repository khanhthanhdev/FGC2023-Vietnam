package org.firstinspires.ftc.teamcode.VNRobot;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Test Shooter")
@Disabled
public class TestShooter extends OpMode {
    private DcMotor shooter;

    Gamepad currentGamepad1 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();

    @Override
    public void init(){
        shooter = hardwareMap.get(DcMotor.class, "leftFront");
    }

    @Override
    public void loop(){
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up){
            shooter.setPower(shooter.getPower() + 0.1);
        } else if (!currentGamepad1.dpad_down && previousGamepad1.dpad_down){
            shooter.setPower(shooter.getPower() - 0.1);
        }

        telemetry.addData("Shooter Power", shooter.getPower());
        telemetry.update();


    }

}
