package frc.lib.subsystems.interfaces;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class VisionInputsAutoLogged extends VisionIO.VisionInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("CameraName", cameraName);
    table.put("CameraPose", cameraPose);
    table.put("FiducialCount", fiducialCount);
    table.put("Ta", ta);
    table.put("TargettingType", targettingType);
    table.put("HasLatestFrame", hasLatestFrame);
    table.put("PoseObservations", poseObservations);
    table.put("TrackedTargets", trackedTargets);
    table.put("CpuTemp", cpuTemp);
    table.put("TotalLatencyMs", totalLatencyMs);
    table.put("Fps", fps);
  }

  @Override
  public void fromLog(LogTable table) {
    cameraName = table.get("CameraName", cameraName);
    cameraPose = table.get("CameraPose", cameraPose);
    fiducialCount = table.get("FiducialCount", fiducialCount);
    ta = table.get("Ta", ta);
    targettingType = table.get("TargettingType", targettingType);
    hasLatestFrame = table.get("HasLatestFrame", hasLatestFrame);
    poseObservations = table.get("PoseObservations", poseObservations);
    trackedTargets = table.get("TrackedTargets", trackedTargets);
    cpuTemp = table.get("CpuTemp", cpuTemp);
    totalLatencyMs = table.get("TotalLatencyMs", totalLatencyMs);
    fps = table.get("Fps", fps);
  }

  public VisionInputsAutoLogged clone() {
    VisionInputsAutoLogged copy = new VisionInputsAutoLogged();
    copy.cameraName = this.cameraName;
    copy.cameraPose = this.cameraPose;
    copy.fiducialCount = this.fiducialCount;
    copy.ta = this.ta;
    copy.targettingType = this.targettingType;
    copy.hasLatestFrame = this.hasLatestFrame;
    copy.poseObservations = this.poseObservations.clone();
    copy.trackedTargets = this.trackedTargets.clone();
    copy.cpuTemp = this.cpuTemp;
    copy.totalLatencyMs = this.totalLatencyMs;
    copy.fps = this.fps;
    return copy;
  }
}
