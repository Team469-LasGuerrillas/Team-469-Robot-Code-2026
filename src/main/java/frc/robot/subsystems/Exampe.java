package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
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

public class Exampe extends SubsystemBase {
  private static Exampe instance;

  private final CanCoderIO canCoder;
  private final CancoderInputsAutoLogged ccInputs = new CancoderInputsAutoLogged();

  private final MotorIO turd;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  public static Exampe createInstance(MotorIO turd, CanCoderIO canCoder) {
    instance = new Exampe(turd, canCoder);
    return instance;
  }

  public static Exampe getInstance() {
    return instance;
  }

  private Exampe(MotorIO turd, CanCoderIO canCoder) {
    this.turd = turd;
    this.canCoder = canCoder;

    turd.setEnableSoftLimits(true, true);
  }

  @Override
  public void periodic() {
    turd.readInputs(talonInputs);
    Logger.processInputs(getName() + " Motor", talonInputs);
    canCoder.readInputs(ccInputs);
    Logger.processInputs(getName() + " CanCoder", ccInputs);
  }

  public void setTargetAngle(Angle angle) {
    turd.setMagicalPositionSetpoint(
        angle, RotationsPerSecond.of(9999), RotationsPerSecondPerSecond.of(9999), 0, 0);
  }

  public void setTargetPoint(Translation2d fieldSpace) {
    Pose2d currentRobotPose = Drive.getInstance().getPose();
    Transform2d turretTransform = GeomUtil.toTransform2d(Constants.ExampeC.TURD_CENTER.toPose2d());
    Pose2d turretFieldPose = currentRobotPose.plus(turretTransform);
    Translation2d turretFieldTranslation = turretFieldPose.getTranslation();

    Rotation2d turretCenterToTarget = fieldSpace.minus(turretFieldTranslation).getAngle();

    Rotation2d deltaRotation = turretCenterToTarget.minus(turretFieldPose.getRotation());

    setTargetAngle(Radians.of(deltaRotation.getRadians()));

    Logger.recordOutput(
        "Turret's target",
        GeomUtil.withRotation(
                currentRobotPose, turretFieldPose.getRotation().plus(new Rotation2d(getPosition())))
            .transformBy(new Transform2d(2, 0, new Rotation2d())));
  }

  public Angle getPosition() {
    return ccInputs.absolutePosition;
  }

  public AngularVelocity getAngularVelocity() {
    return ccInputs.velocity;
  }

  @AutoLogOutput(key = "TurretTarget")
  public Pose2d getTurretTarget() {
    // TODO: Output the theoretical "target" based on current turret angle and flywheel velocity.
    return new Pose2d();
  }
}
