package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
  private boolean turretOverrideLock = false;
  private Angle turretOverrideAngle;

  private int loopCount = 0;

  private double lastTimestamp = Clock.time();

  private Pose2d lastTurretPoseFieldSpace = new Pose2d();
  private SwerveDrivePoseEstimator turretSpeedEstimator =
      new SwerveDrivePoseEstimator(
          Drive.getInstance().kinematics,
          new Rotation2d(),
          Drive.getInstance().lastModulePositions,
          Pose2d.kZero);

  private SwerveDrivePoseEstimator turretTargetSpeedEstimator =
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
            .withMechanismRange(Rotations.of(-0.5), Rotations.of(1.4))
            .withAbsoluteEncoderOffsets(Radians.of(0.510816), Radians.of(2.541806));
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

    turretTargetSpeedEstimator.updateWithTime(
        Clock.time(), new Rotation2d(), Constants.EMPTY_MODULE_POSITIONS);

    double deltaTime = Clock.time() - lastTimestamp;

    double targetAngularVelocityRadiansPerSecond =
        (targetAngle.in(Radians) - lastTargetAngle.in(Radians)) / deltaTime;

    Logger.recordOutput(
        "Turret/TargetAngularVelocity", RadiansPerSecond.of(targetAngularVelocityRadiansPerSecond));

    if (Math.abs(targetAngularVelocityRadiansPerSecond) < Units.degreesToRadians(180)) {
      Matrix<N3, N1> stds = Constants.Field.TURRET_TARGET_SPEEDS_STDS;

      if (Math.abs(targetAngularVelocityRadiansPerSecond) < Units.degreesToRadians(3)) {
        stds = Constants.Field.TURRET_TARGET_SPEEDS_STDS_FOR_ZERO;
      }

      turretTargetSpeedEstimator.addVisionMeasurement(
          GeomUtil.withRotation(
              new Pose2d(), new Rotation2d(targetAngularVelocityRadiansPerSecond)),
          Clock.time(),
          stds);
    }

    lastTimestamp = Clock.time();

    lastTargetAngle = targetAngle;

    lastTurretPoseFieldSpace = getTurretPoseFieldSpace();

    if (!offsetHasBeenSet && getAngle().in(Rotations) != 0) {
      turd.setCurrentPosition(getAngle());
      offsetHasBeenSet = true;
    }

    double wrapOffset =
        Constants.TurretC.WRAPAROUND_PREDICTION_FACOTR
            * talonInputs.motorVelocity.in(RotationsPerSecond);

    boolean nearWrapMax =
        Math.abs(
                Constants.TurretC.TURRERT_MAX.in(Rotations)
                    - talonInputs.motorPosition.in(Rotations))
            < wrapOffset;
    boolean nearWrapMin =
        Math.abs(
                Constants.TurretC.TURRERT_MIN.in(Rotations)
                    - talonInputs.motorPosition.in(Rotations))
            < wrapOffset;

    // boolean targetFar =
    // Math.abs(targetAngle.minus(talonInputs.motorPosition).in(Degrees)) > 40;
    boolean targetFar = Math.abs(targetAngle.minus(talonInputs.motorPosition).in(Degrees)) > 40;

    boolean tooFastTurret = Math.abs(talonInputs.motorVelocity.in(DegreesPerSecond)) > 100;
    boolean tooFastChassis =
        Math.abs(Units.radiansToDegrees(Drive.getInstance().getFieldSpeeds().omegaRadiansPerSecond))
            > 80;

    boolean onTarget =
        (!nearWrapMax
                && !targetFar
                && !nearWrapMin
                && !tooFastTurret
                && !tooFastChassis
                && ToleranceUtil.epsilonEquals(
                    getAngle().in(Rotations),
                    targetAngle.in(Rotations),
                    Constants.TurretC.TURRET_TOLERANCE.in(Rotations)))
            || turretOverrideLock;

    if (onTarget) {
      RobotState.setTurretState(TurretState.LOCKED);
    } else {
      RobotState.setTurretState(TurretState.UNLOCKED);
    }

    loopCount++;
    Logger.recordOutput("LoopCount", loopCount);
  }

  public void lockAngle() {
    turretOverrideLock = true;
    turretOverrideAngle = talonInputs.motorPosition;
  }

  public void setTargetAngle(Angle angle) {

    Angle after = angle;

    // after = Rotations.of(after.in(Rotations) +
    // Math.round(trueTurretRotation.in(Rotations)));

    after = after.plus(Rotations.of(Math.round(talonInputs.motorPosition.in(Rotations))));

    Angle afterPlus = after.plus(Rotations.of(1));
    Angle afterMinus = after.minus(Rotations.of(1));

    // double afterPlusDouble = after.in(Rotations) + 1;
    // double afterMinusDouble = after.in(Rotations) - 1;

    double deltaAfter = Math.abs(after.in(Rotations) - talonInputs.motorPosition.in(Rotations));
    double deltaAfterPlus =
        Math.abs(afterPlus.in(Rotations) - talonInputs.motorPosition.in(Rotations));
    double deltaAfterMinus =
        Math.abs(afterMinus.in(Rotations) - talonInputs.motorPosition.in(Rotations));

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
      // closestAfter = Rotations.of(closestAfter.in(Rotations) - 1);
      closestAfter = closestAfter.minus(Rotations.of(1));
    } else if (closestAfter.in(Rotations) < Constants.TurretC.TURRERT_MIN.in(Rotations)) {
      // closestAfter = Rotations.of(closestAfter.in(Rotations) + 1);
      closestAfter = closestAfter.plus(Rotations.of(1));
    }

    targetAngle = closestAfter;

    closestAfter =
        Rotations.of(
            closestAfter.in(Rotations)
                + (0.053
                    * turretTargetSpeedEstimator
                        .getEstimatedPosition()
                        .getRotation()
                        .getRotations()));

    if (turretOverrideLock) {
      turd.setPositionSetpoint(turretOverrideAngle, RotationsPerSecond.of(0), 0);
    } else {
      turd.setPositionSetpoint(
          closestAfter,
          RotationsPerSecond.of(
              turretTargetSpeedEstimator.getEstimatedPosition().getRotation().getRotations()),
          0);
    }

    Logger.recordOutput(
        "Turret/FilteredTargetAngularVelocity",
        RotationsPerSecond.of(
            turretTargetSpeedEstimator.getEstimatedPosition().getRotation().getRotations()));

    // if (turretOverrideLock) {
    // turd.setMagicalPositionSetpoint(
    // turretOverrideAngle, RotationsPerSecond.of(1),
    // RotationsPerSecondPerSecond.of(3), 0, 0);
    // } else {
    // turd.setMagicalPositionSetpoint(
    // closestAfter,
    // RotationsPerSecond.of(10),
    // RotationsPerSecondPerSecond.of(990),
    // 0,
    // calculateFF());
    // }
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
        "Turret/Crosshair",
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

  @AutoLogOutput(key = "Turret/TurretTargetSpeed")
  public AngularVelocity getTargetAngularVelocity() {
    return RadiansPerSecond.of(
        turretTargetSpeedEstimator.getEstimatedPosition().getRotation().getRadians());
  }

  @AutoLogOutput(key = "Turret/CurrentAngle")
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

  @AutoLogOutput(key = "Turret/TargetAngle")
  public Angle getTurretTarget() {
    return targetAngle;
  }

  @AutoLogOutput(key = "Turret/FF")
  private double calculateFF() {
    return getTargetAngularVelocity().in(DegreesPerSecond) * Constants.TurretC.ROTATION_SPEED_FF;
  }
}
