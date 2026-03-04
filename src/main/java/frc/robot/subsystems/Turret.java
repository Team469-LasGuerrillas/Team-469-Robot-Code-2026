package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.lib.utilities.field.Clock;
import frc.lib.utilities.math.GeomUtil;
import frc.lib.utilities.math.ToleranceUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.TurretState;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class Turret extends SubsystemBase {
  private static Turret instance;

  private final CanCoderIO canCoderA;
  private final CancoderInputsAutoLogged ccAInputs = new CancoderInputsAutoLogged();

  private final CanCoderIO canCoderB;
  private final CancoderInputsAutoLogged ccBInputs = new CancoderInputsAutoLogged();

  private final MotorIO turd;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  private Angle targetAngle = Degrees.of(0);
  private Angle lastTargetAngle = Degrees.of(0);
  private Angle trueTurretRotation = Degrees.of(0);

  private EasyCRTConfig easyCRTConfig;
  private EasyCRT easyCRT;

  private boolean offsetHasBeenSet = false;

  private Pose2d lastTurretPoseFieldSpace = new Pose2d();
  private SwerveDrivePoseEstimator turretSpeedEstimator =
      new SwerveDrivePoseEstimator(
          Drive.getInstance().kinematics,
          new Rotation2d(),
          Drive.getInstance().lastModulePositions,
          Pose2d.kZero);

  private SwerveDrivePoseEstimator turretMotorSpeedEstimator =
      new SwerveDrivePoseEstimator(
          Drive.getInstance().kinematics,
          new Rotation2d(),
          Drive.getInstance().lastModulePositions,
          Pose2d.kZero);

  public static Turret createInstance(MotorIO turd, CanCoderIO canCoderA, CanCoderIO canCoderB) {
    instance = new Turret(turd, canCoderA, canCoderB);
    return instance;
  }

  public static Turret getInstance() {
    return instance;
  }

  private Turret(MotorIO turd, CanCoderIO canCoderA, CanCoderIO canCoderB) {
    this.turd = turd;
    this.canCoderA = canCoderA;
    this.canCoderB = canCoderB;

    turd.setEnableSoftLimits(true, true);

    easyCRTConfig =
        new EasyCRTConfig(this::getCCAPosition, this::getCCBPosition)
            .withEncoderRatios(168.0 / 26.0, 168.0 / 28.0)
            .withAbsoluteEncoder1Inverted(false)
            .withAbsoluteEncoder2Inverted(false)
            .withMatchTolerance(Degrees.of(6.7))
            .withMechanismRange(Rotations.of(-0.99), Rotations.of(0.99))
            .withAbsoluteEncoderOffsets(Radians.of(-1.059981), Radians.of(1.986505));
    easyCRT = new EasyCRT(easyCRTConfig);
  }

  @Override
  public void periodic() {
    turd.readInputs(talonInputs);
    Logger.processInputs(getName() + " Motor", talonInputs);
    canCoderA.readInputs(ccAInputs);
    Logger.processInputs(getName() + " CanCoder A", ccAInputs);
    canCoderB.readInputs(ccBInputs);
    Logger.processInputs(getName() + " CanCoder B", ccBInputs);

    turretSpeedEstimator.updateWithTime(
        Clock.time(), new Rotation2d(), Constants.EMPTY_MODULE_POSITIONS);
    turretSpeedEstimator.addVisionMeasurement(
        GeomUtil.withRotation(
            GeomUtil.toPose2d(
                GeomUtil.toPose2d(getTurretTranslationFieldSpace())
                    .minus(lastTurretPoseFieldSpace)
                    .div(0.02)),
            Rotation2d.fromRadians(
                Drive.getInstance().getFieldSpeedsFiltered().omegaRadiansPerSecond)),
        Clock.time(),
        Constants.Field.TURRET_SPEEDS_STDS);

    turretMotorSpeedEstimator.updateWithTime(
        Clock.time(), new Rotation2d(), Constants.EMPTY_MODULE_POSITIONS);
    turretMotorSpeedEstimator.addVisionMeasurement(
        GeomUtil.withRotation(
            new Pose2d(), new Rotation2d(talonInputs.motorVelocity.in(RadiansPerSecond))),
        Clock.time(),
        Constants.Field.TURRET_MOTOR_SPEEDS_STDS);

    lastTurretPoseFieldSpace = getTurretPoseFieldSpace();

    if (!offsetHasBeenSet && getAngle().in(Rotations) != 0) {
      turd.setCurrentPosition(getAngle());
      offsetHasBeenSet = true;
    }

    double wrapOffset =
        Math.abs(
            Constants.TurretC.WRAPAROUND_PREDICTION_FACOTR
                * talonInputs.motorVelocity.in(RotationsPerSecond));

    boolean nearWrapMax =
        Math.abs(Constants.TurretC.TURRERT_MAX.in(Rotations) - getAngle().in(Rotations))
            < wrapOffset;
    boolean nearWrapMin =
        Math.abs(Constants.TurretC.TURRERT_MIN.in(Rotations) - getAngle().in(Rotations))
            < wrapOffset;

    boolean onTarget =
        !nearWrapMax
            && !nearWrapMin
            && ToleranceUtil.epsilonEquals(
                getAngle().in(Rotations),
                targetAngle.in(Rotations),
                Constants.TurretC.TURRET_TOLERANCE.in(Rotations));

    if (onTarget) {
      RobotState.setTurretState(TurretState.LOCKED);
    } else {
      RobotState.setTurretState(TurretState.UNLOCKED);
    }
  }

  public void setTargetAngle(Angle angle) {

    Angle after =
        angle.minus(
            Rotations.of(
                Constants.TurretC.LEAD_SHOT_OFFSET
                    * Units.radiansToRotations(getTurretSpeedsFieldSpace().omegaRadiansPerSecond)));

    after = after.plus(Rotations.of(Math.round(trueTurretRotation.in(Rotations))));

    Angle afterPlus = after.plus(Rotations.of(1));
    Angle afterMinus = after.minus(Rotations.of(1));

    double deltaAfter = Math.abs(after.in(Rotations) - trueTurretRotation.in(Rotations));
    double deltaAfterPlus = Math.abs(afterPlus.in(Rotations) - trueTurretRotation.in(Rotations));
    double deltaAfterMinus = Math.abs(afterMinus.in(Rotations) - trueTurretRotation.in(Rotations));

    double smallestDelta = Math.min(Math.min(deltaAfterPlus, deltaAfterMinus), deltaAfter);

    Angle closestAfter;

    if (smallestDelta == deltaAfter) {
      closestAfter = after;
    } else if (smallestDelta == deltaAfterPlus) {
      closestAfter = afterPlus;
    } else {
      closestAfter = afterMinus;
    }

    if (closestAfter.in(Rotations) > Constants.TurretC.TURRERT_MAX.in(Rotations)) {
      closestAfter = closestAfter.minus(Rotations.of(1));
    } else if (closestAfter.in(Rotations) < Constants.TurretC.TURRERT_MIN.in(Rotations)) {
      closestAfter = closestAfter.plus(Rotations.of(1));
    }

    Logger.recordOutput("TurretState/Target", closestAfter.in(Rotations));

    targetAngle = closestAfter;

    closestAfter =
        closestAfter.plus(
            Rotations.of(
                0.04
                    * turretMotorSpeedEstimator
                        .getEstimatedPosition()
                        .getRotation()
                        .getRotations()));

    turd.setMagicalPositionSetpoint(
        closestAfter,
        RotationsPerSecond.of(90),
        RotationsPerSecondPerSecond.of(9999),
        0,
        calculateFF());

    lastTargetAngle = closestAfter;
  }

  /**
   * Sets the turret to face an X-Y coordinate in field space.
   *
   * @param fieldSpace The Translation2d X-Y coordinate.
   */
  public void setTargetPoint(Translation2d fieldSpace) {
    Pose2d turretFieldPose = getTurretPoseFieldSpace();
    Translation2d turretFieldTranslation = turretFieldPose.getTranslation();

    Rotation2d turretCenterToTarget = fieldSpace.minus(turretFieldTranslation).getAngle();
    Rotation2d deltaRotation = turretCenterToTarget.minus(turretFieldPose.getRotation());

    setTargetAngle(Radians.of(deltaRotation.getRadians()));

    Logger.recordOutput(
        "Turret's target",
        GeomUtil.withRotation(
                Drive.getInstance().getPose(),
                turretFieldPose.getRotation().plus(new Rotation2d(getAngle())))
            .transformBy(new Transform2d(1, 0, new Rotation2d())));
  }

  /**
   * Returns the turret's pose in field space by applying the turret's robot-relative pose as a
   * transform to the robot's pose.
   *
   * @return The pose of the turret.
   */
  @AutoLogOutput(key = "Turret/FieldPose")
  public Pose2d getTurretPoseFieldSpace() {
    Pose2d currentRobotPose = Drive.getInstance().getPose();
    Transform2d turretTransform = GeomUtil.toTransform2d(Constants.TurretC.TURD_CENTER.toPose2d());
    Pose2d turretFieldPose = currentRobotPose.plus(turretTransform);

    return turretFieldPose;
  }

  public Translation2d getTurretTranslationFieldSpace() {
    Pose2d currentRobotPose = Drive.getInstance().getPose();
    Transform2d turretTransform =
        GeomUtil.toTransform2d(
            GeomUtil.withRotation(Constants.TurretC.TURD_CENTER.toPose2d(), new Rotation2d()));
    Pose2d turretFieldPose = currentRobotPose.plus(turretTransform);

    return turretFieldPose.getTranslation();
  }

  /**
   * Returns the turret's speeds in field space by putting deltas through a kalman filter
   *
   * @return The pose of the turret.
   */
  @AutoLogOutput(key = "Turret/TurretSpeedsFieldSpace")
  public ChassisSpeeds getTurretSpeedsFieldSpace() {
    return GeomUtil.toChassisSpeeds(turretSpeedEstimator.getEstimatedPosition());
  }

  @AutoLogOutput(key = "Turret/Angle")
  public Angle getAngle() {
    if (easyCRT.getAngleOptional().isPresent()) {
      return easyCRT.getAngleOptional().get();
    } else {
      return Rotations.of(0);
    }
  }

  public Angle getAngleForTurretLL() {
    if (getAngle().in(Rotations) != 0) {
      return getAngle();
    } else {
      return talonInputs.motorPosition;
    }
  }

  /**
   * Returns this CCW Positive angle of the primary encoder in turret space.
   *
   * @return
   */
  public Angle getCCAPosition() {
    return ccAInputs.absolutePosition;
  }

  /**
   * Returns this CCW Positive angle of the secondary encoder in turret space.
   *
   * @return
   */
  public Angle getCCBPosition() {
    return ccBInputs.absolutePosition;
  }

  /**
   * Returns the angular velocity of the turret. This can be added onto the robot's angular velocity
   * to figure out the turret's angular velocity in field Space.
   *
   * @return
   */
  public AngularVelocity getAngularVelocity() {
    return talonInputs.motorVelocity;
  }

  @AutoLogOutput(key = "Turret/TurretTarget")
  public Angle getTurretTarget() {
    return targetAngle;
  }

  @AutoLogOutput(key = "Turret/FF")
  private double calculateFF() {
    return Units.radiansToRotations(getTurretSpeedsFieldSpace().omegaRadiansPerSecond)
        * -Constants.TurretC.ROTATION_SPEED_FF;
  }
}
