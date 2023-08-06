package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Log;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.powerplayutil.Height;
import org.firstinspires.ftc.teamcode.Util.ProfiledPIDFController;

import java.util.function.DoubleSupplier;

@Config
public class LiftUpStorage extends SubsystemBase {

    private final MotorEx left, right;
    private final TouchSensor limitSwitch;

    private final double TICKS_IN_DEGREES = 8192.0 / 360;

    public static double kP = 0.0015; //0.0016
    public static double kI = 0.055; //0.06
    public static double kD = 0.0001; //0.00018
    public static double kF = 0.082; //0.06
    public static double maxVelocity = 6000;
    public static double maxAcceleration = 6000;
    private ProfiledPIDFController leftPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    private ProfiledPIDFController rightPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    public static double tolerance = 10;

    public static int currentHeight = 0;

    public static int manualLiftSpeed = 30;

    public static int threshold = 30;

    public LiftUpStorage(MotorEx left, MotorEx right, TouchSensor limitSwitch) {
        this.limitSwitch = limitSwitch;
        this.left = left;
        this.right = right;
        leftPIDF.setTolerance(tolerance);
        leftPIDF.setGoal(0);
        rightPIDF.setGoal(0);
    }

    private void setHeight(Height height) {
        currentHeight = height.getHeight();
        leftPIDF.setGoal(height.getHeight());
        rightPIDF.setGoal(height.getHeight());
    }

    private void setHeight(int tick) {
        currentHeight = tick;
        leftPIDF.setGoal(tick);
        rightPIDF.setGoal(tick);
    }

    public int getLeftEncoderValue() {
        return left.getCurrentPosition();
    }

    public boolean atTarget() {
        return left.getCurrentPosition() < currentHeight + threshold &&
                left.getCurrentPosition() > currentHeight - threshold;
    }

    public int getCurrentGoal() {
        return currentHeight;
    }

    public Command goTo(Height height) {
        return new InstantCommand(() -> setHeight(height))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command goTo(int tick) {
        return new InstantCommand(() -> setHeight(tick))
                .andThen(new WaitUntilCommand(this::atTarget));
    }

    public Command setPower(DoubleSupplier power) {
        return new RunCommand(() -> {
            if(Math.abs(power.getAsDouble()) > 0.01) {
                left.set(power.getAsDouble() / 2);
                right.set(power.getAsDouble() / 2);
                leftPIDF.setGoal(left.getCurrentPosition());
                rightPIDF.setGoal(right.getCurrentPosition());
            }
        }, this);
    }

    @Override
    public void periodic() {
        if (limitSwitch.isPressed()) {
            left.resetEncoder();
            right.resetEncoder();
        }

        double output_left = leftPIDF.calculate(left.getCurrentPosition());
        double output_right = rightPIDF.calculate(right.getCurrentPosition());
        left.set(output_left);
//        Log.d("asd", "output left: "+ output_left);
        right.set(output_right);
    }

    public void setVelocityAccel(double maxVelocity, double maxAcceleration){
        leftPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
        rightPIDF = new ProfiledPIDFController(kP, kI, kD, kF,
                new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration), TICKS_IN_DEGREES);
    }
}