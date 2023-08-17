package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name="Gamepad State")
public class GamepadState extends LinearOpMode {

    private DcMotor intake;
    boolean intakeToggle = false;

    @Override
    public void runOpMode() {
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        intake = hardwareMap.get(DcMotor.class, "intake");




        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            // Rising edge detector
            if (currentGamepad1.circle && !previousGamepad1.circle) {
                intakeToggle = !intakeToggle;
            }

            // Using the toggle variable to control the robot.
            if (intakeToggle) {
                intake.setPower(1);
            }
            else {
                intake.setPower(0);
            }


        }
    }
}
