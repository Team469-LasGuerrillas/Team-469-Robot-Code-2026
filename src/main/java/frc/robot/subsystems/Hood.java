package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.lib.utilities.math.ToleranceUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HoodState;
import frc.robot.util.FieldZoning;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static Hood instance;

  private final MotorIO hoodMotor;
  private final MotorInputsAutoLogged pivotInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  private Angle requestedAngle = Radians.of(0);

  private boolean zeroed = false;
  private boolean disabledLimits = false;

  public static Hood createinstance(MotorIO pivotMotor) {
    instance = new Hood(pivotMotor);
    return instance;
  }

  public static Hood getInstance() {
    return instance;
  }

  private Hood(MotorIO pivotMotor) {
    this.hoodMotor = pivotMotor;
  }

  public void resetHood() {
    zeroed = false;
    disabledLimits = false;
  }

  @Override
  public void periodic() {
    hoodMotor.readInputs(pivotInputs);
    Logger.processInputs(getName() + " Motor", pivotInputs);

    Angle updatedRequestedAngle = requestedAngle;

    if (FieldZoning.retractHood()) {
      updatedRequestedAngle = Constants.HoodC.HOOD_STOW;
      Logger.recordOutput("Hood/RetractHood", true);
    } else {
      Logger.recordOutput("Hood/RetractHood", false);
    }

    if (!disabledLimits) {
      hoodMotor.setEnableSoftLimits(false, false);
      hoodMotor.setZero();
      disabledLimits = true;
    }

    if (!zeroed) {
      hoodMotor.setMagicalPositionSetpoint(
          Rotations.of(-1),
          RotationsPerSecond.of(0.1),
          RotationsPerSecondPerSecond.of(0.1),
          0,
          calcFF(requestedAngle));
    } else {
      hoodMotor.setMagicalPositionSetpoint(
          updatedRequestedAngle,
          RotationsPerSecond.of(9999),
          RotationsPerSecondPerSecond.of(9999),
          0,
          calcFF(requestedAngle));
    }

    if (!zeroed
        && Math.abs(pivotInputs.motorVelocity.in(DegreesPerSecond)) < 5
        && pivotInputs.statorCurrent.in(Amps) > 14) {
      hoodMotor.setZero();
      zeroed = true;
      hoodMotor.setEnableSoftLimits(true, true);
    }

    boolean lowHood =
        ToleranceUtil.epsilonEquals(
            getCurrentAngle().in(Rotations),
            Constants.HoodC.HOOD_STOW.in(Rotations),
            Units.degreesToRotations(0.05));
    boolean highHood =
        ToleranceUtil.epsilonEquals(
            getCurrentAngle().in(Rotations),
            Constants.HoodC.HOOD_MAX.in(Rotations),
            Units.degreesToRotations(0.2));

    boolean onTarget =
        !lowHood
            && !highHood
            && ToleranceUtil.epsilonEquals(
                getCurrentAngle().in(Rotations),
                getTargetAngle().in(Rotations),
                Constants.HoodC.HOOD_TOLERANCE.in(Rotations));

    if (onTarget) {
      RobotState.setHoodState(HoodState.LOCKED);
    } else {
      RobotState.setHoodState(HoodState.UNLOCKED);
    }
  }

  @AutoLogOutput(key = "Hood/TargetAngle")
  public Angle getTargetAngle() {
    return requestedAngle;
  }

  public void setTargetAngle(Angle newAngleRequest) {
    requestedAngle = newAngleRequest;
  }

  public Angle getCurrentAngle() {
    return pivotInputs.motorPosition;
  }

  private double calcFF(Angle pivotAngle) {
    return 0;
  }
}
