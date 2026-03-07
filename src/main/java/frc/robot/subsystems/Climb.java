package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static Climb instance;

  private final MotorIO climbMotor;
  private final MotorInputsAutoLogged climbInputs = new MotorInputsAutoLogged();

  private double requestedDutyCycle = 0;

  public static Climb createInstance(MotorIO climbMotor) {
    instance = new Climb(climbMotor);
    return instance;
  }

  public static Climb GetInstance() {
    return instance;
  }

  private Climb(MotorIO climbMotor) {
    this.climbMotor = climbMotor;

    climbMotor.setEnableSoftLimits(true, true);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutyCycle = dutyCycle;
    climbMotor.setOpenLoopDutyCycle(dutyCycle);
  }

  public void setPosition(Angle position) {
    climbMotor.setMagicalPositionSetpoint(
        position, RotationsPerSecond.of(9999), RotationsPerSecondPerSecond.of(0), 0, 0);
  }

  @Override
  public void periodic() {
    climbMotor.readInputs(climbInputs);
    Logger.processInputs(getName() + "Climb Motor", climbInputs);
  }
}
