package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TurretedCamera {
  public static Pose3d recalcPose(Rotation2d turretRotation, Pose3d originalPose, Pose3d turretCenter) {
    Transform3d centerToOriginalCamera = originalPose.minus(turretCenter);

    turretCenter.rotateBy(new Rotation3d(turretRotation));

    return turretCenter.plus(centerToOriginalCamera);
  }
}
