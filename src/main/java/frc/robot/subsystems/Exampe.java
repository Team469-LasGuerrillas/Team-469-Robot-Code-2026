package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Exampe extends SubsystemBase {
  private final MotorIO motor;
  private final MotorInputsAutoLogged inputs = new MotorInputsAutoLogged();

  public Exampe(MotorIO motor) {
    this.motor = motor;
  }

  @Override
  public void periodic() {
    motor.readInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }
}
