package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.VisionIO;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservationComparator;
import frc.lib.subsystems.interfaces.VisionIO.TargettingType;
import frc.lib.subsystems.interfaces.VisionInputsAutoLogged;
import frc.lib.utilities.field.Clock;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.util.FiducialFilters;
import frc.robot.subsystems.vision.util.FiducialFilters.FiducialModifications;
import frc.robot.subsystems.vision.util.TurretedCamera;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Function;
import java.util.function.UnaryOperator;
import org.littletonrobotics.junction.Logger;

public class FiducialVision extends SubsystemBase {
  private static List<PoseObservation> allObservations = new LinkedList<>();

  private final VisionIO io;
  private final VisionInputsAutoLogged visionInputs = new VisionInputsAutoLogged();

  private Pose3d originalCameraPose;
  private boolean hasOriginalPoseBeenSet = false;

  ArrayList<Function<PoseObservation, Boolean>> extraRejections;
  ArrayList<UnaryOperator<FiducialModifications>> extraModifications;

  public static void applyUpdates() {
    allObservations.sort(new PoseObservationComparator());

    for (int i = 0; i < allObservations.size(); i++) {
      PoseObservation o = allObservations.get(i);

      Logger.recordOutput("Data into pose estimator delay", Clock.time() - o.timestamp());

      Drive.getInstance()
          .addVisionMeasurement(
              o.pose().toPose2d(),
              o.timestamp(),
              VecBuilder.fill(o.stdDevs()[0], o.stdDevs()[1], o.stdDevs()[2]));
    }
  }

  public FiducialVision(
      VisionIO io,
      ArrayList<Function<PoseObservation, Boolean>> extraRejections,
      ArrayList<UnaryOperator<FiducialModifications>> extraModifications) {
    this.io = io;
    this.extraRejections = extraRejections;
    this.extraModifications = extraModifications;
    // Set tag filter override
  }

  @Override
  public void periodic() {
    io.readInputs(visionInputs);
    io.setRobotRotationUpdate(
        Drive.getInstance().getRotation(),
        RadiansPerSecond.of(Drive.getInstance().getFieldSpeeds().omegaRadiansPerSecond));
    Logger.processInputs(getCameraName(), visionInputs);

    if (!hasOriginalPoseBeenSet) {
      originalCameraPose = visionInputs.cameraPose;
      hasOriginalPoseBeenSet = true;
    }

    List<PoseObservation> robotPosesAccepted = new LinkedList<>();
    List<PoseObservation> robotPosesRejected = new LinkedList<>();

    if (visionInputs.hasLatestFrame
        && visionInputs.targettingType == TargettingType.FIDUCIAL
        && visionInputs.fiducialCount >= 1) {
      for (PoseObservation observation : visionInputs.poseObservations) {
        // TODO: Apply rejection
        boolean reject =
            FiducialFilters.FiducialRejections.badAmbiguity(observation)
                || FiducialFilters.FiducialRejections.badYaw(observation)
                || FiducialFilters.FiducialRejections.hasNoTags(observation)
                || FiducialFilters.FiducialRejections.isOffField(observation)
                || FiducialFilters.FiducialRejections.isFlying(observation)
                || FiducialFilters.FiducialRejections.tooSmall(observation);

        boolean supplementalReject = false;
        for (Function<PoseObservation, Boolean> rejection : extraRejections) {
          if (rejection.apply(observation)) {
            supplementalReject = true;
          }
        }

        if (reject || supplementalReject) {
          robotPosesRejected.add(observation);
        } else {
          robotPosesAccepted.add(observation);

          FiducialModifications filteredObservation =
              new FiducialFilters.FiducialModifications(observation).withUpdateYaw();

          for (UnaryOperator<FiducialModifications> modification : extraModifications) {
            filteredObservation = modification.apply(filteredObservation);
          }

          allObservations.add(filteredObservation.get());
        }
      }
    }

    Logger.recordOutput(
        getCameraName() + "/RobotPosesAccepted",
        robotPosesAccepted.toArray(new PoseObservation[robotPosesAccepted.size()]));

    Logger.recordOutput(
        getCameraName() + "/RobotPosesRejected",
        robotPosesRejected.toArray(new PoseObservation[robotPosesRejected.size()]));
  }

  public void setPoseRobotSpace(Pose3d pose) {
    io.setPoseRobotSpace(pose);
  }

  public void setPositionTurret(Angle turretAngle, Pose3d turretCenter) {
    Pose3d updatedPose = TurretedCamera.recalcPose(turretAngle, originalCameraPose, turretCenter);
    io.setPoseRobotSpace(updatedPose);
  }

  public String getCameraName() {
    return visionInputs.cameraName;
  }
}
