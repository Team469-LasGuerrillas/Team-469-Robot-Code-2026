package frc.lib.utilities.math;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShootTarget;

/**
 * Helpers for biasing shooter setpoints when the chassis is tipped.
 *
 * <p>This keeps the public methods intentionally simple: pass the setpoint you would have used on
 * flat carpet and get the setpoint to apply while tipped.
 */
public final class AngledShotSetpointHelper {
  private static final double TARGET_ENTRANCE_HEIGHT_INCHES = 67.0;
  private static final double SHOOTER_EXIT_HEIGHT_INCHES = 18.0;
  private static final double TARGET_HEIGHT_ABOVE_SHOOTER_FLAT_INCHES =
      TARGET_ENTRANCE_HEIGHT_INCHES - SHOOTER_EXIT_HEIGHT_INCHES;

  private static final double TRACK_WIDTH_INCHES = 23.75;
  private static final double WHEELBASE_LENGTH_INCHES = 19.75;
  private static final double MIN_COMPENSATION_TILT_RADIANS = Math.toRadians(2.0);

  private AngledShotSetpointHelper() {}

  /**
   * Adjusts a robot-relative turret setpoint for chassis pitch/roll.
   *
   * <p>The returned angle is the original setpoint plus the difference between the flat
   * robot-relative target bearing and the tipped robot-relative target bearing.
   */
  public static Rotation2d compensateTurretForRobotAngle(Rotation2d flatTurretSetpoint) {
    if (!isRobotMoreThanTwoDegreesFromNormal()) {
      return flatTurretSetpoint;
    }

    Rotation2d flatBearing = getFlatRobotRelativeBearingToTarget();
    Translation3d targetRobotSpace = getTargetVectorRobotSpaceInches();
    Rotation2d tippedBearing = new Rotation2d(targetRobotSpace.getX(), targetRobotSpace.getY());

    return flatTurretSetpoint.plus(tippedBearing.minus(flatBearing));
  }

  /**
   * Adjusts the hood angle, in degrees, for chassis pitch/roll.
   *
   * <p>This applies the change in apparent target elevation caused by the chassis tilt and the lift
   * of the chassis center while resting on one or more wheels.
   */
  public static double compensateHoodForRobotAngle(double flatHoodDegrees) {
    if (!isRobotMoreThanTwoDegreesFromNormal()) {
      return flatHoodDegrees;
    }

    double flatDistanceInches = ShootTarget.getDistanceToTarget().in(Inches);
    double flatElevationRadians =
        Math.atan2(TARGET_HEIGHT_ABOVE_SHOOTER_FLAT_INCHES, flatDistanceInches);

    Translation3d targetRobotSpace = getTargetVectorRobotSpaceInches();
    double tippedDistanceInches = Math.hypot(targetRobotSpace.getX(), targetRobotSpace.getY());
    double tippedElevationRadians = Math.atan2(targetRobotSpace.getZ(), tippedDistanceInches);

    return flatHoodDegrees - Math.toDegrees(tippedElevationRadians - flatElevationRadians);
  }

  /**
   * Adjusts the flywheel setpoint, in rotations per second, for chassis pitch/roll.
   *
   * <p>Without a projectile-speed-per-RPS calibration constant, the safest one-argument correction
   * is a range ratio. If your lookup table already handles this better, keep this method as a small
   * feedforward bias or replace the body with another lookup at the tipped effective range.
   */
  public static double compensateFlywheelForRobotAngle(double flatFlywheelRps) {
    if (!isRobotMoreThanTwoDegreesFromNormal()) {
      return flatFlywheelRps;
    }

    double flatDistanceInches = ShootTarget.getDistanceToTarget().in(Inches);
    double flatRangeInches =
        Math.hypot(flatDistanceInches, TARGET_HEIGHT_ABOVE_SHOOTER_FLAT_INCHES);

    Translation3d targetRobotSpace = getTargetVectorRobotSpaceInches();
    double tippedRangeInches =
        Math.sqrt(
            targetRobotSpace.getX() * targetRobotSpace.getX()
                + targetRobotSpace.getY() * targetRobotSpace.getY()
                + targetRobotSpace.getZ() * targetRobotSpace.getZ());

    if (flatRangeInches <= 1e-9) {
      return flatFlywheelRps;
    }

    return flatFlywheelRps * (tippedRangeInches / flatRangeInches);
  }

  private static boolean isRobotMoreThanTwoDegreesFromNormal() {
    double rollAboutRobotX = Drive.getInstance().getPitch().in(Radians);
    double pitchAboutRobotY = Drive.getInstance().getRoll().in(Radians);

    double normalZ = Math.cos(rollAboutRobotX) * Math.cos(pitchAboutRobotY);
    double tiltFromNormalRadians = Math.acos(clamp(normalZ, -1.0, 1.0));

    return tiltFromNormalRadians > MIN_COMPENSATION_TILT_RADIANS;
  }

  private static Rotation2d getFlatRobotRelativeBearingToTarget() {
    Translation2d targetField = ShootTarget.getTranslationToTarget();
    Translation2d turretField = Turret.getInstance().getTurretPoseFieldSpace().getTranslation();

    Rotation2d fieldBearing = targetField.minus(turretField).getAngle();
    return fieldBearing.minus(Drive.getInstance().getRotation());
  }

  private static Translation3d getTargetVectorRobotSpaceInches() {
    Pose2d turretPose = Turret.getInstance().getTurretPoseFieldSpace();
    Translation2d targetField = ShootTarget.getTranslationToTarget();

    Rotation2d fieldBearing = targetField.minus(turretPose.getTranslation()).getAngle();
    double flatDistanceInches = ShootTarget.getDistanceToTarget().in(Inches);

    Translation3d targetFieldSpace =
        new Translation3d(
            flatDistanceInches * fieldBearing.getCos(),
            flatDistanceInches * fieldBearing.getSin(),
            TARGET_HEIGHT_ABOVE_SHOOTER_FLAT_INCHES - getChassisCenterLiftInches());

    return fieldVectorToTiltedRobotVector(targetFieldSpace);
  }

  private static Translation3d fieldVectorToTiltedRobotVector(Translation3d fieldVector) {
    double chassisYaw = Drive.getInstance().getRotation().getRadians();

    // These accessors are intentionally swapped in the robot code:
    // getPitch() returns roll about robot X, getRoll() returns pitch about robot Y.
    double rollAboutRobotX = Drive.getInstance().getPitch().in(Radians);
    double pitchAboutRobotY = Drive.getInstance().getRoll().in(Radians);

    Translation3d yawOnlyRobotVector = rotateZ(fieldVector, -chassisYaw);
    Translation3d unpitchedVector = rotateY(yawOnlyRobotVector, -pitchAboutRobotY);
    return rotateX(unpitchedVector, -rollAboutRobotX);
  }

  private static double getChassisCenterLiftInches() {
    double rollAboutRobotX = Drive.getInstance().getPitch().in(Radians);
    double pitchAboutRobotY = Drive.getInstance().getRoll().in(Radians);

    double halfLength = WHEELBASE_LENGTH_INCHES / 2.0;
    double halfWidth = TRACK_WIDTH_INCHES / 2.0;

    double minWheelZ = Double.POSITIVE_INFINITY;
    for (double x : new double[] {-halfLength, halfLength}) {
      for (double y : new double[] {-halfWidth, halfWidth}) {
        Translation3d wheelAfterTilt =
            rotateY(rotateX(new Translation3d(x, y, 0.0), rollAboutRobotX), pitchAboutRobotY);
        minWheelZ = Math.min(minWheelZ, wheelAfterTilt.getZ());
      }
    }

    return Math.max(0.0, -minWheelZ);
  }

  private static Translation3d rotateX(Translation3d vector, double radians) {
    double cos = Math.cos(radians);
    double sin = Math.sin(radians);
    return new Translation3d(
        vector.getX(),
        vector.getY() * cos - vector.getZ() * sin,
        vector.getY() * sin + vector.getZ() * cos);
  }

  private static Translation3d rotateY(Translation3d vector, double radians) {
    double cos = Math.cos(radians);
    double sin = Math.sin(radians);
    return new Translation3d(
        vector.getX() * cos + vector.getZ() * sin,
        vector.getY(),
        -vector.getX() * sin + vector.getZ() * cos);
  }

  private static Translation3d rotateZ(Translation3d vector, double radians) {
    double cos = Math.cos(radians);
    double sin = Math.sin(radians);
    return new Translation3d(
        vector.getX() * cos - vector.getY() * sin,
        vector.getX() * sin + vector.getY() * cos,
        vector.getZ());
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
