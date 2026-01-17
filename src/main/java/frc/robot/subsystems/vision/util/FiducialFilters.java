package frc.robot.subsystems.vision.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservationType;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class FiducialFilters {
  public static class FiducialRejections {
    public static boolean hasNoTags(PoseObservation toFilter) {
      return toFilter.tagCount() == 0;
    }

    public static boolean badAmbiguity(PoseObservation toFilter) {
      return (toFilter.tagCount() == 1
          && toFilter.ambiguity() > Constants.VisionC.MAX_SINGLE_TAG_AMBIGUITY);
    }

    public static boolean tooSmall(PoseObservation toFilter) {
      return (toFilter.tagCount() == 1
          && toFilter.type() == PoseObservationType.MT1
          && toFilter.ta() < Constants.VisionC.MIN_SINGLE_TAG_AREA);
    }

    public static boolean badYaw(PoseObservation toFilter) {
      if (toFilter.type() == PoseObservationType.MT1 && toFilter.tagCount() == 1) {
        return toFilter
                .pose()
                .getRotation()
                .getMeasureZ()
                .minus(Drive.getInstance().getRotation().getMeasure())
                .abs(Degrees)
            <= Constants.VisionC.MAX_YAW_ERROR_MT1.in(Degrees);
      } else if (toFilter.type() == PoseObservationType.MT2) {
        return toFilter
                .pose()
                .getRotation()
                .getMeasureZ()
                .minus(Drive.getInstance().getRotation().getMeasure())
                .abs(Degrees)
            <= Constants.VisionC.MAX_YAW_ERROR_MT2.in(Degrees);
      }
      return true;
    }

    public static boolean isFlying(PoseObservation toFilter) {
      if (Math.abs(toFilter.pose().getZ()) >= Constants.VisionC.MAX_FLOATING_NOCLIP.in(Meters)) {
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
        observation.stdDevs()[2] *= 6.7;
      }
      return this;
    }

    public PoseObservation get() {
      return observation;
    }
  }
}
