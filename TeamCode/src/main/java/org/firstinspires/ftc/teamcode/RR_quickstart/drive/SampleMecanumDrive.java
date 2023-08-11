package org.firstinspires.ftc.teamcode.RR_quickstart.drive;

import static org.firstinspires.ftc.teamcode.Math.Controllers.CriticallyDampedPDControl.solveKD;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.RR_quickstart.drive.DriveConstants.encoderTicksToInches;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.RR_quickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RR_quickstart.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.RR_quickstart.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.RR_quickstart.util.MedianFilter3;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class SampleMecanumDrive extends MecanumDrive {
	public static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
	public static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);
	public static final TrajectoryVelocityConstraint VEL_CONSTRAINT_FAST_TURN = getVelocityConstraint(MAX_VEL, Math.toRadians(400), TRACK_WIDTH);

	public static PIDCoefficients TRANSLATIONAL_PID;
	public static PIDCoefficients HEADING_PID; // new PIDCoefficients(5, 0, 0.3);
	public static double LATERAL_MULTIPLIER = 1;
	public static double VX_WEIGHT = 1;
	public static double VY_WEIGHT = 1;
	public static double OMEGA_WEIGHT = 1;
	static double translation_kp = 14;
	static double rotation_Kp = 6;

	static {
		try {
			TRANSLATIONAL_PID = new PIDCoefficients(translation_kp, 0, solveKD(translation_kp, DriveConstants.kV, DriveConstants.kA));
		} catch (Exception e) {
			TRANSLATIONAL_PID = new PIDCoefficients(11, 0, 3);
			System.out.println("controller synthesis failed, reverting to safe coefficients");
			e.printStackTrace();
		}
	}

	static {
		try {
			HEADING_PID = new PIDCoefficients(rotation_Kp, 0, solveKD(rotation_Kp, DriveConstants.kV / TRACK_WIDTH, DriveConstants.gyrationConstant * DriveConstants.kA / TRACK_WIDTH));
		} catch (Exception e) {
			HEADING_PID = new PIDCoefficients(rotation_Kp, 0, 0);
			System.out.println("heading controller synthesis failed, reverting to safe coefficients");
		}
	}

	public boolean isHoldingPosition = false;
	public Pose2d holdingPose = new Pose2d();
	MedianFilter3 voltageFilter = new MedianFilter3();
	TwoWheelTrackingLocalizer localizer2Wheel;
	private final BNO055IMU imu;
	private PIDFController axialController = new PIDFController(TRANSLATIONAL_PID);
	private PIDFController lateralController = new PIDFController(TRANSLATIONAL_PID);
	private PIDFController headingController = new PIDFController(HEADING_PID);
	private final TrajectorySequenceRunner trajectorySequenceRunner;
	private final TrajectoryFollower follower;
	private final DcMotorEx leftFront;
	private final DcMotorEx leftRear;
	private final DcMotorEx rightRear;
	private final DcMotorEx rightFront;
	private final List<DcMotorEx> motors;
	//    private BNO055IMU imu;
	private final VoltageSensor batteryVoltageSensor;

	public SampleMecanumDrive(HardwareMap hardwareMap) {
		super(DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

		headingController.setInputBounds(-Math.PI, Math.PI);

		follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
				new Pose2d(0.1, 0.1, Math.toRadians(0.3)), 1);

		LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

		batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


		// TODO: adjust the names of the following hardware devices to match your configuration
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		parameters.mode = BNO055IMU.SensorMode.NDOF;
		imu.initialize(parameters);


		// TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
		// not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
		//
		//             | +Z axis
		//             |
		//             |
		//             |
		//      _______|_____________     +Y axis
		//     /       |_____________/|__________
		//    /   REV / EXPANSION   //
		//   /       / HUB         //
		//  /_______/_____________//
		// |_______/_____________|/
		//        /
		//       / +X axis
		//
		// This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
		// and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
		//
		// For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
		// BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);


		leftFront = hardwareMap.get(DcMotorEx.class, "FrontLeft");
		leftRear = hardwareMap.get(DcMotorEx.class, "BackLeft");
		rightRear = hardwareMap.get(DcMotorEx.class, "BackRight");
		rightFront = hardwareMap.get(DcMotorEx.class, "FrontRight");


		motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

		for (DcMotorEx motor : motors) {
			MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
			motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
			motor.setMotorType(motorConfigurationType);
		}

		if (RUN_USING_ENCODER) {
			setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		}

		setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
			setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
		}

		// TODO: reverse any motors using DcMotor.setDirection()
		// TODO: reverse any motors using DcMotor.setDirection()
		leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
		leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

		trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
	}

	public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
		return new MinVelocityConstraint(Arrays.asList(
				new AngularVelocityConstraint(maxAngularVel),
				new MecanumVelocityConstraint(maxVel, trackWidth)
		));
	}

	public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
		return new ProfileAccelerationConstraint(maxAccel);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
		return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
		return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
		return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
	}

	public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
		return new TrajectorySequenceBuilder(
				startPose,
				VEL_CONSTRAINT, ACCEL_CONSTRAINT,
				MAX_ANG_VEL, MAX_ANG_ACCEL
		);
	}

	public void turnAsync(double angle) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder(getPoseEstimate())
						.turn(angle)
						.build()
		);
	}

	public void turn(double angle) {
		turnAsync(angle);
		waitForIdle();
	}

	public void followTrajectoryAsync(Trajectory trajectory) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(
				trajectorySequenceBuilder(trajectory.start())
						.addTrajectory(trajectory)
						.build()
		);
	}

	public void followTrajectory(Trajectory trajectory) {
		followTrajectoryAsync(trajectory);
		waitForIdle();
	}

	public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
		trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
	}

	public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
		followTrajectorySequenceAsync(trajectorySequence);
		waitForIdle();
	}

	public Pose2d getLastError() {
		return trajectorySequenceRunner.getLastPoseError();
	}

	public void update() {
		updatePoseEstimate();
		DriveSignal signal;
		if (!isHoldingPosition) {
			signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
		} else {
			signal = poseStabilize(holdingPose);
		}
		if (signal != null) setDriveSignal(signal);
	}

	public void waitForIdle() {
		while (!Thread.currentThread().isInterrupted() && isBusy())
			update();
	}

	public boolean isBusy() {
		return trajectorySequenceRunner.isBusy();
	}

	public void setMode(DcMotor.RunMode runMode) {
		for (DcMotorEx motor : motors) {
			motor.setMode(runMode);
		}
	}

	public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
		for (DcMotorEx motor : motors) {
			motor.setZeroPowerBehavior(zeroPowerBehavior);
		}
	}

	public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
		PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
				coefficients.p, coefficients.i, coefficients.d,
				coefficients.f * 12 / batteryVoltageSensor.getVoltage()
		);

		for (DcMotorEx motor : motors) {
			motor.setPIDFCoefficients(runMode, compensatedCoefficients);
		}
	}

	public void setWeightedDrivePower(Pose2d drivePower) {
		Pose2d vel = drivePower;

		if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
				+ Math.abs(drivePower.getHeading()) > 1) {
			// re-normalize the powers according to the weights
			double denom = VX_WEIGHT * Math.abs(drivePower.getX())
					+ VY_WEIGHT * Math.abs(drivePower.getY())
					+ OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

			vel = new Pose2d(
					VX_WEIGHT * drivePower.getX(),
					VY_WEIGHT * drivePower.getY(),
					OMEGA_WEIGHT * drivePower.getHeading()
			).div(denom);
		}

		setDrivePower(vel);
	}

	@NonNull
	@Override
	public List<Double> getWheelPositions() {
		List<Double> wheelPositions = new ArrayList<>();
		for (DcMotorEx motor : motors) {
			wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
		}
		return wheelPositions;
	}

	@Override
	public List<Double> getWheelVelocities() {
		List<Double> wheelVelocities = new ArrayList<>();
		for (DcMotorEx motor : motors) {
			wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
		}
		return wheelVelocities;
	}

	@Override
	public void setMotorPowers(double v, double v1, double v2, double v3) {
		double voltage = batteryVoltageSensor.getVoltage();
		double scaleFactor = 12 / voltage;
		leftFront.setPower(v * scaleFactor);
		leftRear.setPower(v1 * scaleFactor);
		rightRear.setPower(v2 * scaleFactor);
		rightFront.setPower(v3 * scaleFactor);
	}

	@Override
	public double getRawExternalHeading() {
		return imu.getAngularOrientation().firstAngle;
	}

	@Override
	public Double getExternalHeadingVelocity() {

		return (double) imu.getAngularVelocity().zRotationRate;

	}

	/**
	 * convert the 3.3 to 0 voltage into an angle
	 *
	 * @param voltage voltage from the sensor
	 * @return converted angle
	 */
	public double voltageToAngle(double voltage) {
		double computedAngle = ((voltage / 3.3) * 360);
		computedAngle = Math.toRadians(computedAngle);
//        computedAngle = AngleWrap(computedAngle + initialAngle);

		return -computedAngle;
	}

	@NonNull
	public List<Double> getWheelPos() {
		return localizer2Wheel.getWheelPositions();
	}

	public DriveSignal poseStabilize(Pose2d targetPose) {
		Pose2d poseError = Kinematics.calculateRobotPoseError(targetPose, getPoseEstimate());
		axialController.setTargetPosition(poseError.getX());
		lateralController.setTargetPosition(poseError.getY());
		headingController.setTargetPosition(poseError.getHeading());
		Pose2d robotVelocity = getPoseVelocity();

		if (robotVelocity == null) {
			robotVelocity = new Pose2d();
		}

		double axialCorrection = axialController.update(0.0, robotVelocity.getX());
		double lateralCorrection = lateralController.update(0.0, robotVelocity.getY());
		double headingCorrection = headingController.update(0.0, robotVelocity.getHeading());
		return new DriveSignal(new Pose2d(axialCorrection, lateralCorrection, headingCorrection), new Pose2d(0, 0, 0));

	}

	public void setCoefficients(boolean followingTrajectory) {
		if (followingTrajectory) {
			axialController = new PIDFController(TRANSLATIONAL_PID);
			lateralController = new PIDFController(TRANSLATIONAL_PID);
			headingController = new PIDFController(HEADING_PID);
		} else {
			axialController = new PIDFController(new PIDCoefficients(60,0,solveKD(60, DriveConstants.kV, DriveConstants.kA)));
			lateralController = new PIDFController(new PIDCoefficients(60,0,solveKD(60, DriveConstants.kV, DriveConstants.kA)));
			headingController = new PIDFController(new PIDCoefficients(60,0,solveKD(60, DriveConstants.kV/ TRACK_WIDTH, DriveConstants.kA/ TRACK_WIDTH)));
		}
	}


}
