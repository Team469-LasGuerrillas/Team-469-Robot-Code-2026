package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.VisionIO;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservationComparator;
import frc.lib.subsystems.interfaces.VisionIO.TargettingType;
import frc.lib.subsystems.interfaces.VisionInputsAutoLogged;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.util.FiducialFilters;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FiducialVision extends SubsystemBase {
  private static List<PoseObservation> allObservations = new LinkedList<>();

  private final VisionIO io;
  private final VisionInputsAutoLogged visionInputs = new VisionInputsAutoLogged();

  public static void applyUpdates() {
    allObservations.sort(new PoseObservationComparator());

    for (int i = 0; i < allObservations.size(); i++) {
      PoseObservation o = allObservations.get(i);

      Drive.getInstance()
          .addVisionMeasurement(
              o.pose().toPose2d(),
              o.timestamp(),
              VecBuilder.fill(o.stdDevs()[0], o.stdDevs()[1], o.stdDevs()[2]));
    }
  }

  public FiducialVision(VisionIO io) {
    this.io = io;
    // Set tag filter override
  }

  @Override
  public void periodic() {
    io.readInputs(visionInputs);
    io.setRobotRotationUpdate(
        Drive.getInstance().getRotation(),
        RadiansPerSecond.of(Drive.getInstance().getFieldSpeeds().omegaRadiansPerSecond));
    Logger.processInputs(getCameraName(), visionInputs);

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
                || FiducialFilters.FiducialRejections.tooSmall(observation);

        if (reject) {
          robotPosesRejected.add(observation);
        } else {
          robotPosesAccepted.add(observation);

          // TODO: Apply more filters
          PoseObservation filteredObservation =
              new FiducialFilters.FiducialModifications(observation).withUpdateYaw().get();

          allObservations.add(filteredObservation);
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

  public String getCameraName() {
    return visionInputs.cameraName;
  }
}
