package frc.lib.utilities.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ShootAndMove {
  public static Translation2d getTransformed(
      ChassisSpeeds fieldRelativeRobotSpeeds,
      Pose2d currentPose,
      Translation2d target,
      Pose3d turretPose,
      boolean isPassing) {
    Distance dist = Meters.of(target.getDistance(currentPose.getTranslation()));

    double lastLoopTimeOfFlight = 0;
    double thisLoopTimeOfFlight;

    if (isPassing) {
      thisLoopTimeOfFlight = Constants.LauncherC.TIME_OF_FLIGHT_MAP_PASSING.get(dist.in(Meters));
    } else {
      thisLoopTimeOfFlight = Constants.LauncherC.TIME_OF_FLIGHT_MAP_SHOOTING.get(dist.in(Meters));
    }
    int loopCount = 0;
    Translation2d returnTarget = new Translation2d();

    double turretDist = Math.hypot(turretPose.getX(), turretPose.getY());
    double turretTangentVelocity = fieldRelativeRobotSpeeds.omegaRadiansPerSecond * turretDist;

    Angle turretTangentAngle =
        Radians.of(
            Math.atan2(turretPose.getY(), turretPose.getX())
                + 0.5 * Math.PI
                + currentPose.getRotation().getRadians());

    double turretTangentX = turretTangentVelocity * Math.cos(turretTangentAngle.in(Radians));
    double turretTangentY = turretTangentVelocity * Math.sin(turretTangentAngle.in(Radians));

    while (Math.abs((lastLoopTimeOfFlight - thisLoopTimeOfFlight) / thisLoopTimeOfFlight) > 0.01
        || loopCount < 1) {
      lastLoopTimeOfFlight = thisLoopTimeOfFlight;

      Translation2d offset =
          new Translation2d(
              lastLoopTimeOfFlight * (fieldRelativeRobotSpeeds.vxMetersPerSecond + turretTangentX),
              lastLoopTimeOfFlight * (fieldRelativeRobotSpeeds.vyMetersPerSecond + turretTangentY));

      returnTarget = target.minus(offset);
      Distance updatedDist = Meters.of(returnTarget.getDistance(currentPose.getTranslation()));

      if (isPassing) {
        thisLoopTimeOfFlight = Constants.LauncherC.TIME_OF_FLIGHT_MAP_PASSING.get(dist.in(Meters));
      } else {
        thisLoopTimeOfFlight = Constants.LauncherC.TIME_OF_FLIGHT_MAP_SHOOTING.get(dist.in(Meters));
      }
      loopCount++;

      if (loopCount > 300) {
        break;
      }
    }

    return returnTarget;
  }
}
