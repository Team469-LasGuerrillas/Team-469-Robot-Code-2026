package frc.lib.subsystems.interfaces;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Temperature;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {

  public record TrackedTarget(
      double timestamp,
      Pose3d origin,
      Angle tx,
      Angle ty,
      double area,
      TargettingType targetType) {}

  public record PoseObservation(
      double timestamp,
      double ambiguity,
      double ta,
      double tagCount,
      Pose3d pose,
      double[] stdDevs,
      PoseObservationType type) {}

  public enum TargettingType {
    FIDUCIAL,
    COLOR,
    RETROREFLECTIVE,
    OBJECT_DETECTION
  }

  public enum PoseObservationType {
    MT1,
    MT2
  }

  @AutoLog
  public class VisionInputs {
    // General targetting
    public int fiducialCount = 0;
    public double ta = 0;
    public TargettingType targettingType = TargettingType.RETROREFLECTIVE;
    public boolean hasLatestFrame = false;

    // Apriltag localization
    public PoseObservation[] poseObservations;

    // Object detection
    public TrackedTarget[] trackedTargets;

    // Performance
    public Temperature cpuTemp = Temperature.ofBaseUnits(0, null);
    public double totalLatencyMs = 0;
    public double fps = 0;
  }

  default void readInputs(VisionInputsAutoLogged inputs) {}

  default void setPoseRobotSpace(Pose3d cameraPose) {}

  default void setRobotRotationUpdate(Rotation2d rotation, Rotation2d angularVelocity) {}

  default void setPipelineIndex(int index) {}

  default void setTagFiltersOverride(int[] tags) {}
}
