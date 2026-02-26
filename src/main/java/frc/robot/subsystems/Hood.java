package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.lib.utilities.math.ToleranceUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.HoodState;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private static Hood instance;

  private final MotorIO hoodMotor;
  private final MotorInputsAutoLogged pivotInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  private Angle requestedAngle = Radians.of(0);

  public static Hood createinstance(MotorIO pivotMotor) {
    instance = new Hood(pivotMotor);
    return instance;
  }

  public static Hood getInstance() {
    return instance;
  }

  private Hood(MotorIO pivotMotor) {
    this.hoodMotor = pivotMotor;

    pivotMotor.setZero();
    pivotMotor.setEnableSoftLimits(true, true);
  }

  @Override
  public void periodic() {
    hoodMotor.readInputs(pivotInputs);
    Logger.processInputs(getName() + " Motor", pivotInputs);

    hoodMotor.setMagicalPositionSetpoint(
        requestedAngle,
        RotationsPerSecond.of(9999),
        RotationsPerSecondPerSecond.of(9999),
        0,
        calcFF(requestedAngle));

    boolean onTarget =
        ToleranceUtil.epsilonEquals(
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
