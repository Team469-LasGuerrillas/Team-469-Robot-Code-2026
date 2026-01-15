package frc.robot.subsystems.vision.util;

import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservationType;

public class FiducialFilters {
  public static class FiducialRejections {

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
