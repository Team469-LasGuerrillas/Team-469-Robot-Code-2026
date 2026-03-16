package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.utilities.field.Station;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class FieldZoning {
  public static boolean inNeutralZone() {
    double turretCurrentPoseX = Turret.getInstance().getTurretPoseFieldSpace().getX();
    boolean isRed = Station.isRed();

    if (turretCurrentPoseX > Constants.Field.BLUE_TRENCH_SCORING.in(Meters) && !isRed) {
      return true;
    } else if (turretCurrentPoseX < Constants.Field.RED_TRENCH_SCORING.in(Meters) && isRed) {
      return true;
    }
    return false;
  }

  public static Translation2d dynamicPassLocation() {
    // This has 0 being Y mid line
    double turretCurrentPoseY =
        Turret.getInstance().getTurretPoseFieldSpace().getY()
            - Constants.Field.MID_FIELD_Y.in(Meters);
    boolean isRed = Station.isRed();
    double targetY;

    boolean fromTop = turretCurrentPoseY > 0;

    if (fromTop) {
      targetY =
          Constants.Field.MAX_FIELD_Y.in(Meters)
              - 1.469
              - 0.4
                  * Constants.Field.MID_FIELD_Y.in(Meters)
                  * (turretCurrentPoseY / Constants.Field.MID_FIELD_Y.in(Meters));
    } else {
      targetY =
          1.469
              - 0.4
                  * Constants.Field.MID_FIELD_Y.in(Meters)
                  * (turretCurrentPoseY / Constants.Field.MID_FIELD_Y.in(Meters));
    }

    if (isRed) {
      return new Translation2d(
          Constants.Field.MAX_FIELD_X.minus(Constants.Field.PASS_WALL_DISTANCE).in(Meters),
          targetY);
    } else {
      return new Translation2d(Constants.Field.PASS_WALL_DISTANCE.in(Meters), targetY);
    }
  }

  public static boolean retractHood() {
    Pose2d turretCurrentPose = Turret.getInstance().getTurretPoseFieldSpace();
    ChassisSpeeds turretSpeedsFieldSpace = Turret.getInstance().getTurretSpeedsFieldSpace();

    double xDeltaBlue =
        Math.abs(turretCurrentPose.getX() - Constants.Field.BLUE_TRENCH_SCORING.in(Meters));
    double xDeltaRed =
        Math.abs(turretCurrentPose.getX() - Constants.Field.RED_TRENCH_SCORING.in(Meters));

    // Blue
    boolean blueFromRight =
        turretSpeedsFieldSpace.vxMetersPerSecond < 0
            && turretCurrentPose.getX() > Constants.Field.BLUE_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.MID_FIELD_X.in(Meters)
            && turretCurrentPose.getX() > 0;
    boolean blueFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.BLUE_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.MID_FIELD_X.in(Meters)
            && turretCurrentPose.getX() > 0;

    // Red

    boolean redFromRight =
        turretSpeedsFieldSpace.vxMetersPerSecond < 0
            && turretCurrentPose.getX() > Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() > Constants.Field.MID_FIELD_X.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.MAX_FIELD_X.in(Meters);
    boolean redFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() > Constants.Field.MID_FIELD_X.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.MAX_FIELD_X.in(Meters);

    double updatedZoneBlue = Constants.Field.REGULAR_DECAPITATION_ZONE.in(Meters);
    double updatedZoneRed = Constants.Field.REGULAR_DECAPITATION_ZONE.in(Meters);

    if (blueFromLeft || blueFromRight) {
      updatedZoneBlue +=
          Constants.Field.DECAPITATION_SPEED_FACTOR
              * Math.abs(turretSpeedsFieldSpace.vxMetersPerSecond);
    }

    if (redFromLeft || redFromRight) {
      updatedZoneRed +=
          Constants.Field.DECAPITATION_SPEED_FACTOR
              * Math.abs(turretSpeedsFieldSpace.vxMetersPerSecond);
    }

    if (xDeltaBlue < updatedZoneBlue || xDeltaRed < updatedZoneRed) {
      return true;
    }

    return false;
  }
}
