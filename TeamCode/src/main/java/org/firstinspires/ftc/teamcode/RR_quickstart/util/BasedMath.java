package org.firstinspires.ftc.teamcode.RR_quickstart.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class BasedMath {


	/**
	 * given a global pose, offset it by robot relative positions
	 *
	 * @param poseGlobal     global pose
	 * @param robotRelativeX forward direction relative to the robot
	 * @param robotRelativeY side direction relative to the robot. (+) left (-) right
	 * @return shifted pose.
	 */
	public static Pose2d shiftRobotRelative(Pose2d poseGlobal, double robotRelativeX, double robotRelativeY) {
		// unit vector of the global Pose
		Vector2d globalVec = poseGlobal.vec();
		Vector2d globalVecUnit = poseGlobal.headingVec();
		// orthogonal vec
		Vector2d orthogonalVec = new Vector2d(-globalVecUnit.getY(), globalVecUnit.getX());

		Vector2d combinedVector = globalVec.plus(globalVecUnit.times(robotRelativeX))
				.plus(orthogonalVec.times(robotRelativeY));
		return new Pose2d(combinedVector, poseGlobal.getHeading());
	}

}
