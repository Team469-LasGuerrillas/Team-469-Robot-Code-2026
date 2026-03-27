package frc.robot.subsystems.vision.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservationType;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import java.util.function.UnaryOperator;

public class FiducialFilters {
  public static class FiducialRejections {
    public static boolean hasNoTags(PoseObservation toFilter) {
      return toFilter.tagCount() == 0;
    }

    public static boolean badAmbiguity(PoseObservation toFilter) {
      return (toFilter.tagCount() == 1
          && toFilter.type() == PoseObservationType.MT1
          && toFilter.ambiguity() > Constants.VisionC.MAX_SINGLE_TAG_AMBIGUITY);
    }

    public static boolean badTurretAngularVelocity(PoseObservation toFilter) {
      return (Turret.getInstance().getAngularVelocity().abs(DegreesPerSecond)
          >= Constants.VisionC.BAD_TURRET_ANGULAR_VELOCITY.abs(DegreesPerSecond));
    }

    public static boolean tooSmall(PoseObservation toFilter) {
      return (toFilter.tagCount() == 1
          && toFilter.type() == PoseObservationType.MT1
          && toFilter.ta() < Constants.VisionC.MIN_SINGLE_TAG_AREA);
    }

    /** Is the MT2 or single tag yaw far off from where the robot thinks it is? */
    public static boolean badYaw(PoseObservation toFilter) {
      Rotation2d rotationAtTimestamp;
      if (Drive.getInstance().getPose(toFilter.timestamp()).isPresent()) {
        rotationAtTimestamp = Drive.getInstance().getPose(toFilter.timestamp()).get().getRotation();
      } else {
        rotationAtTimestamp = Drive.getInstance().getRotation();
      }
      if (toFilter.type() == PoseObservationType.MT1 && toFilter.tagCount() == 1) {
        // MT1 1 Tag Cases
        return toFilter
                .pose()
                .getRotation()
                .getMeasureZ()
                .minus(rotationAtTimestamp.getMeasure())
                .abs(Degrees)
            >= Constants.VisionC.MAX_YAW_ERROR_MT1.in(Degrees);
      } else if (toFilter.type() == PoseObservationType.MT2) {
        // MT2 Case
        return toFilter
                .pose()
                .getRotation()
                .getMeasureZ()
                .minus(rotationAtTimestamp.getMeasure())
                .abs(Degrees)
            >= Constants.VisionC.MAX_YAW_ERROR_MT2.in(Degrees);
      }
      // MT1 2 Tag Case
      return false;
    }

    public static boolean isOffField(PoseObservation toFilter) {
      return (toFilter.pose().getX() > Constants.Field.MAX_FIELD_X.in(Meters)
          || toFilter.pose().getY() > Constants.Field.MAX_FIELD_Y.in(Meters)
          || toFilter.pose().getX() < 0
          || toFilter.pose().getY() < 0);
    }

    /** Is the robot flying? If so, return false. */
    public static boolean isFlying(PoseObservation toFilter) {
      if (Math.abs(toFilter.pose().getZ()) >= Constants.VisionC.MAX_FLOATING_NOCLIP.in(Meters)
      /*&& (toFilter.type() == PoseObservationType.MT2
      || toFilter.tagCount() == 1 && toFilter.type() == PoseObservationType.MT1)*/ ) {
        return true;
      }
      return false;
    }
  }

  public static class FiducialModifications {
    private final PoseObservation observation;

    public FiducialModifications(PoseObservation toFilter) {
      this.observation = toFilter;
    }

    public FiducialModifications withUpdateYaw() {
      if (observation.type() == PoseObservationType.MT2) {
        observation.stdDevs()[2] = Double.MAX_VALUE;
      } else {
        if (DriverStation.isEnabled()) {
          observation.stdDevs()[2] *= 6.7;
        }
      }
      return this;
    }

    public FiducialModifications withMultiplyAllResults() {
      observation.stdDevs()[0] *= 1.0;
      observation.stdDevs()[1] *= 1.0;

      return this;
    }

    public FiducialModifications withDistrustMt2WhileTurretSpinToFast() {
      if (observation.type() == PoseObservationType.MT2
          && Turret.getInstance().getAngularVelocity().abs(DegreesPerSecond)
              >= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT2.abs(DegreesPerSecond)) {
        observation.stdDevs()[0] *= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT2_MULT;
        observation.stdDevs()[1] *= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT2_MULT;
      } else if (observation.type() == PoseObservationType.MT1
          && Turret.getInstance().getAngularVelocity().abs(DegreesPerSecond)
              >= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT1.abs(DegreesPerSecond)) {
        observation.stdDevs()[0] *= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT1_MULT;
        observation.stdDevs()[1] *= Constants.VisionC.REASONABLE_TURRET_ANGULAR_VELOCITY_MT1_MULT;
        observation.stdDevs()[2] = Double.MAX_VALUE;
      }
      return this;
    }

    public FiducialModifications withDistrustMt2WhileDriveSpinToFast() {
      if (observation.type() == PoseObservationType.MT2
          && Units.radiansToDegrees(Drive.getInstance().getFieldSpeeds().omegaRadiansPerSecond)
              >= Constants.VisionC.REASONABLE_DRIVE_ANGULAR_VELOCITY_MT2.abs(DegreesPerSecond)) {
        observation.stdDevs()[0] *= Constants.VisionC.REASONABLE_DRIVE_ANGULAR_VELOCITY_MT2_MULT;
        observation.stdDevs()[1] *= Constants.VisionC.REASONABLE_DRIVE_ANGULAR_VELOCITY_MT2_MULT;
      }
      return this;
    }

    public FiducialModifications withDistrustMt1() {
      if (observation.type() == PoseObservationType.MT1) {
        observation.stdDevs()[0] = 99999;
        observation.stdDevs()[1] = 99999;
        observation.stdDevs()[2] = 99999;
      }
      return this;
    }

    public FiducialModifications withDistrustYaw() {
      observation.stdDevs()[2] = Double.MAX_VALUE;
      return this;
    }

    public PoseObservation get() {
      return observation;
    }

    public static UnaryOperator<FiducialModifications> o_withMultiplyAllResults() {
      return FiducialModifications::withMultiplyAllResults;
    }

    public static UnaryOperator<FiducialModifications> o_withDistrustMt2WhileTurretSpinToFast() {
      return FiducialModifications::withDistrustMt2WhileTurretSpinToFast;
    }

    public static UnaryOperator<FiducialModifications> o_withDistrustMt2WhileDriveSpinToFast() {
      return FiducialModifications::withDistrustMt2WhileDriveSpinToFast;
    }

    public static UnaryOperator<FiducialModifications> o_withDistrustMt1() {
      return FiducialModifications::withDistrustMt1;
    }

    public static UnaryOperator<FiducialModifications> o_withDistrustYaw() {
      return FiducialModifications::withDistrustYaw;
    }
  }
}
