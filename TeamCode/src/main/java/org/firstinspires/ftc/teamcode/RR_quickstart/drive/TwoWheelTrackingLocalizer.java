package org.firstinspires.ftc.teamcode.RR_quickstart.drive;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RR_quickstart.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    |           || |
 *    |           || |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer {
	public static double TICKS_PER_REV = 8192;
	public static double WHEEL_RADIUS = (35 / 2.0) / 25.4; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	public static double xScale = 1;//72/72.8;
	public static double yScale = 1;//72/71.74105985588855;

	public static double PARALLEL_X = 1; // X is the up and down direction
	public static double PARALLEL_Y = 10.509951133427611 / 2.0; // Y is the strafe direction

	public static double PERPENDICULAR_X = 1.6;
	public static double PERPENDICULAR_Y = -2;

	// Parallel/Perpendicular to the forward axis
	// Parallel wheel is parallel to the forward axis
	// Perpendicular is perpendicular to the forward axis
	private final Encoder parallelEncoder;
	private final Encoder perpendicularEncoder;

	private final SampleMecanumDrive drive;

	public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, SampleMecanumDrive drive) {
		super(Arrays.asList(
				new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
				new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
		));

		this.drive = drive;

		parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "FrontLeft"));
		perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "BackRight"));
		perpendicularEncoder.setDirection(Encoder.Direction.FORWARD);

		// TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
	}

	public static double encoderTicksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	@Override
	public double getHeading() {
		return drive.getRawExternalHeading();
	}

	@Override
	public Double getHeadingVelocity() {
		return drive.getExternalHeadingVelocity();
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		double parallel = parallelEncoder.getCurrentPosition();
		double perpendicular = perpendicularEncoder.getCurrentPosition();
		return Arrays.asList(
				encoderTicksToInches(parallel * xScale),
				encoderTicksToInches(perpendicular * yScale)
		);
	}

	@NonNull
	@Override
	public List<Double> getWheelVelocities() {
		// TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
		//  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
		//  compensation method

		return Arrays.asList(
				encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
				encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
		);
	}
}

