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
            && turretCurrentPose.getY() > Constants.Field.BLUE_SIDE_MIN.in(Meters)
            && turretCurrentPose.getY() < Constants.Field.MAX_FIELD_Y.in(Meters);
    boolean blueFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.BLUE_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getY() > Constants.Field.BLUE_SIDE_MIN.in(Meters)
            && turretCurrentPose.getY() < Constants.Field.MAX_FIELD_Y.in(Meters);



    // Red

    boolean redFromRight =
        turretSpeedsFieldSpace.vxMetersPerSecond < 0
            && turretCurrentPose.getX() > Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getY() < Constants.Field.BLUE_SIDE_MIN.in(Meters)
            && turretCurrentPose.getY() > 0;
    boolean redFromLeft =
        turretSpeedsFieldSpace.vxMetersPerSecond > 0
            && turretCurrentPose.getX() < Constants.Field.RED_TRENCH_SCORING.in(Meters)
            && turretCurrentPose.getY() < Constants.Field.BLUE_SIDE_MIN.in(Meters)
            && turretCurrentPose.getY() > 0;


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
