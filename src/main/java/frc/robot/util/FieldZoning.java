package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class FieldZoning {
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
            && turretCurrentPose.getX() < Constants.Field.FIELD_MIDDLE.in(Meters)
            && turretCurrentPose.getX() > 0;
    boolean blueFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.BLUE_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.FIELD_MIDDLE.in(Meters)
            && turretCurrentPose.getX() > 0;

    // Red

    boolean redFromRight =
        turretSpeedsFieldSpace.vxMetersPerSecond < 0
            && turretCurrentPose.getX() > Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() > Constants.Field.FIELD_MIDDLE.in(Meters)
            && turretCurrentPose.getX() < Constants.Field.MAX_FIELD_X.in(Meters);
    boolean redFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getX() > Constants.Field.FIELD_MIDDLE.in(Meters)
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
