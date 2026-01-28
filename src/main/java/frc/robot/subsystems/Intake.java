package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private static Intake instance;

  private final MotorIO intakeRoller;
  private final MotorIO intakePivot;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
  private final CancoderInputsAutoLogged ccInputs = new CancoderInputsAutoLogged();

  private final CanCoderIO canCoder;

  private double requestedDutycycle = 0;

  private Angle requestedAngle = Units.Degrees.of(0);

  public Intake createinstance() {
    return instance;
  }

  public static Intake Getinstance() {
    return instance;
  }

  private Intake(MotorIO intakeMotor, MotorIO intakeMotor2, CanCoderIO canCoder) {
    this.intakeRoller = intakeMotor;
    this.intakePivot = intakeMotor;
    this.canCoder = canCoder;

    intakeRoller.setEnableSoftLimits(true, true);
    intakePivot.setEnableSoftLimits(true, true);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    intakeRoller.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public void setTargetAngle(Angle newAngleRequest) {
    requestedAngle = newAngleRequest;
    intakePivot.setMagicalPositionSetpoint(
        newAngleRequest, RotationsPerSecond.of(1), RotationsPerSecondPerSecond.of(1), 0, 0);
  }

  @Override
  public void periodic() {
    intakeRoller.readInputs(talonInputs);
    Logger.processInputs(getName() + "Roller Motor", talonInputs);
    intakePivot.readInputs(talonInputs);
    Logger.processInputs(getName() + "Pivot Motor", talonInputs);
    canCoder.readInputs(ccInputs);
    Logger.processInputs(getName() + "CanCoder", ccInputs);
  }
}
