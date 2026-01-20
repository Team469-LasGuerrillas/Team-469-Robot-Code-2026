package frc.robot.subsystems.vision.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;

public class TurretedCamera {
  public static Pose3d recalcPose(Angle turretRotation, Pose3d originalPose, Pose3d turretCenter) {
    Rotation2d turretRotation2d = new Rotation2d(turretRotation.in(Radians));

    Transform3d centerToOriginalCamera = originalPose.minus(turretCenter);

    turretCenter.rotateBy(new Rotation3d(turretRotation2d));

    return turretCenter.plus(centerToOriginalCamera);
  }
}
