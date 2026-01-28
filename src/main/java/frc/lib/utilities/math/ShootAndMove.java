package frc.lib.utilities.math;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import frc.lib.utilities.field.ShooterRegression;

public class ShootAndMove {
  public static Translation2d getTransformed(
      ChassisSpeeds fieldRelativeRobotSpeeds, Pose2d currentPose, Translation2d target) {
    Distance dist = Meters.of(target.getDistance(currentPose.getTranslation()));

    double lastLoopTimeOfFlight = 0;
    double thisLoopTimeOfFlight =
        dist.in(Meters) / ShooterRegression.calculateSpeedAndAngle(dist)[0]; // In meters per second
    int loopCount = 0;
    Translation2d returnTarget = new Translation2d();

    while (Math.abs((lastLoopTimeOfFlight - thisLoopTimeOfFlight) / thisLoopTimeOfFlight) > 0.01
        || loopCount < 1) {
      lastLoopTimeOfFlight = thisLoopTimeOfFlight;

      Translation2d offset =
          new Translation2d(
              lastLoopTimeOfFlight * fieldRelativeRobotSpeeds.vxMetersPerSecond,
              lastLoopTimeOfFlight * fieldRelativeRobotSpeeds.vyMetersPerSecond);

      returnTarget = target.minus(offset);
      Distance updatedDist = Meters.of(returnTarget.getDistance(currentPose.getTranslation()));

      thisLoopTimeOfFlight =
          updatedDist.in(Meters) / ShooterRegression.calculateSpeedAndAngle(updatedDist)[0];
      loopCount++;

      if (loopCount > 300) {
        return null;
      }
    }

    return returnTarget;
  }
}
