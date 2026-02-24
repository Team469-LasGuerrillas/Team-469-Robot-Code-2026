package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.lib.utilities.math.GeomUtil;
import frc.robot.Constants;
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
  private int numRotations = 0;
  private Angle previousCancoderPosition = Degrees.of(0);
  private Angle trueTurretRotation = Degrees.of(0);

  private EasyCRTConfig easyCRTConfig;
  private EasyCRT easyCRT;

  private boolean offsetHasBeenSet;

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

    if (!offsetHasBeenSet && getAngle().in(Rotations) != 0) {
      turd.setCurrentPosition(getAngle());
      offsetHasBeenSet = true;
    }

    /*/
    // Get Cancoder Value [0.5-0.5) Rotations
    double cancoderRotationValue = ccAInputs.absolutePosition.in(Rotations);

    // Check if we wrapped
    if (cancoderRotationValue - previousCancoderPosition.in(Rotations) >= 0.5) {
      // Yes, we wrapped pos->neg (+1 rotation)
      numRotations--;
    } else if (cancoderRotationValue - previousCancoderPosition.in(Rotations) <= -0.5) {
      // Yes, we wrapped neg->pos (-1 rotation)
      numRotations++;
    }

    // System.out.println(numRotations);

    // Calculate actual turret rotation
    trueTurretRotation = ccAInputs.absolutePosition.plus(Rotations.of(numRotations));

    // Save previous
    previousCancoderPosition = Rotations.of(cancoderRotationValue);
    */

    trueTurretRotation = getAngle();

    Logger.recordOutput("TurretState/Position", trueTurretRotation.in(Rotations));
  }

  public void setTargetAngle(Angle angle) {
    targetAngle = angle;

    Angle after = angle;

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

    turd.setMagicalPositionSetpoint(
        closestAfter,
        RotationsPerSecond.of(9999),
        RotationsPerSecondPerSecond.of(9999),
        0,
        0);
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
  public Pose2d getTurretPoseFieldSpace() {
    Pose2d currentRobotPose = Drive.getInstance().getPose();
    Transform2d turretTransform = GeomUtil.toTransform2d(Constants.TurretC.TURD_CENTER.toPose2d());
    Pose2d turretFieldPose = currentRobotPose.plus(turretTransform);

    return turretFieldPose;
  }

  @AutoLogOutput(key = "Turret/Angle")
  public Angle getAngle() {
    if (easyCRT.getAngleOptional().isPresent()) {
      return easyCRT.getAngleOptional().get();
    } else {
      return Rotations.of(0);
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
    return ccAInputs.velocity;
  }

  @AutoLogOutput(key = "TurretTarget")
  public Pose2d getTurretTarget() {
    // TODO: Output the theoretical "target" based on current turret angle and
    // flywheel velocity.
    return new Pose2d();
  }
}
