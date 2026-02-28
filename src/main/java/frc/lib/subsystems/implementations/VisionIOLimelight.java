package frc.lib.subsystems.implementations;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.drivers.LimelightHelpers;
import frc.lib.drivers.LimelightHelpers.RawDetection;
import frc.lib.drivers.LimelightHelpers.RawFiducial;
import frc.lib.subsystems.interfaces.VisionIO;
import frc.lib.subsystems.interfaces.VisionInputsAutoLogged;
import frc.lib.utilities.field.Clock;
import frc.robot.subsystems.drive.Drive;
import java.util.HashMap;

/*
                  .:------:.
              .-===-:::::--=+=-.
            :=+-.            .-+=:
         .=+:    -+**##**=.    :=+:
       .=+.   .:-:    :+#####.      =+.
     :+-   .#########*    *##: :%*    -+:
  .-=+.    *####+-.       .#*  -##=    :+=-:
+=        -%*:  .:            =####.        =+
+-        -#. .+#*           -####*         -+
  -=+=.     .####.        =####*=.     .=+=-
     =+:    -####:  *+:    ..         :+-
      :+-   :####=  =%#*=-:..:-=*:   :+:
       .+=   =####.  *########%#-   -+:
          :+=.  :+##.  -=++=-   .-+-
              :=+=-:......:-=+=:
                 .:-======-:.
*/

public class VisionIOLimelight implements VisionIO {

  static HashMap<String, VisionIOLimelight> instances = new HashMap<String, VisionIOLimelight>();

  private final String limelightName;
  private double lastHeartbeat = -1;

  // RawFiducial[] fiducials;
  RawDetection[] detections;

  private double totalLatencyMs;
  private Pose3d cameraPose;

  private VisionIOLimelight(String limelightName, Pose3d cameraPose) {
    this.limelightName = limelightName;

    // fiducials = LimelightHelpers.getRawFiducials(limelightName);
    detections = LimelightHelpers.getRawDetections(limelightName);

    setPoseRobotSpace(cameraPose);
  }

  public static VisionIOLimelight getInstance(String limelightName, Pose3d cameraPose) {
    if (instances.get(limelightName) == null) {
      instances.put(limelightName, new VisionIOLimelight(limelightName, cameraPose));
    }
    return instances.get(limelightName);
  }

  @Override
  public void readInputs(VisionInputsAutoLogged inputs) {
    inputs.cameraName = limelightName;
    inputs.cameraPose = cameraPose; // LimelightHelpers.getCameraPose3d_RobotSpace(limelightName);

    inputs.fiducialCount = LimelightHelpers.getTargetCount(limelightName);
    inputs.ta = LimelightHelpers.getTA(limelightName);
    inputs.targettingType = getPipeType();
    double heartbeat = LimelightHelpers.getLimelightNTDouble(limelightName, "hb");
    inputs.hasLatestFrame = lastHeartbeat < heartbeat;
    lastHeartbeat = heartbeat;

    inputs.totalLatencyMs =
        LimelightHelpers.getLatency_Capture(limelightName)
            + LimelightHelpers.getLatency_Pipeline(limelightName);
    totalLatencyMs = inputs.totalLatencyMs;

    inputs.poseObservations = parsePoseObservations();
    inputs.trackedTargets = parseTargets();

    double[] hw =
        NetworkTableInstance.getDefault()
            .getTable(limelightName)
            .getEntry("hw")
            .getDoubleArray(new double[4]);
    inputs.cpuTemp = Celsius.of(hw[1]);
    inputs.fps = hw[0];
  }

  @Override
  public void setPoseRobotSpace(Pose3d cameraPose) {
    this.cameraPose = cameraPose;
    LimelightHelpers.setCameraPose_RobotSpace(
        limelightName,
        cameraPose.getX(),
        -cameraPose.getY(),
        cameraPose.getZ(),
        Math.toDegrees(cameraPose.getRotation().getX()),
        Math.toDegrees(cameraPose.getRotation().getY()),
        Math.toDegrees(cameraPose.getRotation().getZ()));
  }

  @Override
  public void setRobotRotationUpdate(Rotation2d rotation, AngularVelocity angularVelocity) {
    LimelightHelpers.SetRobotOrientation(
        limelightName, rotation.getDegrees(), angularVelocity.in(DegreesPerSecond), 0, 0, 0, 0);
  }

  @Override
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(limelightName, index);
  }

  @Override
  public void setTagFiltersOverride(int[] tags) {
    LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, tags);
  }

  private TargettingType getPipeType() {
    String pipeString = LimelightHelpers.getCurrentPipelineType(limelightName);

    TargettingType type;

    switch (pipeString) {
      case "pipe_fiducial":
        type = TargettingType.FIDUCIAL;
        break;

      case "pipe_color":
        type = TargettingType.COLOR;

      case "pipe_neuralnetwork":
        type = TargettingType.OBJECT_DETECTION;

      default:
        type = TargettingType.RETROREFLECTIVE;
        break;
    }

    return type;
  }

  private TrackedTarget[] parseTargets() {
    detections = LimelightHelpers.getRawDetections(limelightName);

    int numberOfTargets = detections.length;
    TrackedTarget[] targets = new TrackedTarget[numberOfTargets];

    for (int i = 0; i < detections.length; i++) {
      targets[i] =
          new TrackedTarget(
              Clock.time() - Units.millisecondsToSeconds(totalLatencyMs),
              new Pose3d(Drive.getInstance().getPose())
                  .plus(
                      new Transform3d(
                          cameraPose.getX(),
                          cameraPose.getY(),
                          cameraPose.getZ(),
                          cameraPose.getRotation())),
              Degrees.of(detections[i].txnc),
              Degrees.of(detections[i].tync),
              detections[i].ta,
              getPipeType());
    }

    return targets;
  }

  private PoseObservation[] parsePoseObservations() {
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(limelightName, "stddevs");
    RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials(limelightName);

    if (stddevs.length == 0) {
      return new PoseObservation[0];
    }

    double[] stddevsMt1 = new double[] {stddevs[0], stddevs[1], stddevs[5]};
    double[] stddevsMt2 = new double[] {stddevs[6], stddevs[7], stddevs[11]};

    if (stddevs.length == 0 || rawFiducials.length == 0) {
      return new PoseObservation[] {};
    }

    LimelightHelpers.PoseEstimate mt1Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    LimelightHelpers.PoseEstimate mt2Estimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    PoseObservation[] poses = {
      new PoseObservation(
          Clock.time() - Units.millisecondsToSeconds(mt1Estimate.latency),
          rawFiducials[0].ambiguity,
          mt1Estimate.avgTagArea,
          mt1Estimate.tagCount,
          LimelightHelpers.getBotPose3d_wpiBlue(limelightName),
          stddevsMt1,
          PoseObservationType.MT1),
      new PoseObservation(
          Clock.time() - Units.millisecondsToSeconds(mt1Estimate.latency),
          rawFiducials[0].ambiguity,
          mt2Estimate.avgTagArea,
          mt2Estimate.tagCount,
          LimelightHelpers.getBotPose3d_wpiBlue_MegaTag2(limelightName),
          stddevsMt2,
          PoseObservationType.MT2)
    };

    return poses;
  }

  @Override
  public void setThrottle(int value) {
    NetworkTableInstance.getDefault().getTable(limelightName).getEntry("throttle_set").setNumber(value);
  }
}
