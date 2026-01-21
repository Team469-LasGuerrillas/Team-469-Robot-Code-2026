package frc.robot.subsystems.vision.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public class TurretedCamera {
  public static Pose3d recalcPose(Angle turretRotation, Pose3d originalPose, Pose3d turretCenter) {
    Rotation2d turretRotation2d = new Rotation2d(turretRotation);

    Transform3d centerToOriginalCamera = originalPose.minus(turretCenter);

    Pose3d rotatedTurretCenter =
        turretCenter.transformBy(
            new Transform3d(new Transform2d(new Translation2d(), turretRotation2d)));

    return rotatedTurretCenter.plus(centerToOriginalCamera);
  }
}
