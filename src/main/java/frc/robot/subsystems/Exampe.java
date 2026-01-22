package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Exampe extends SubsystemBase {
  private static Exampe instance;

  private final CanCoderIO canCoder;
  private final CancoderInputsAutoLogged inputs = new CancoderInputsAutoLogged();

  public static Exampe createInstance(CanCoderIO canCoder) {
    instance = new Exampe(canCoder);
    return instance;
  }

  public static Exampe getInstance() {
    return instance;
  }

  private Exampe(CanCoderIO canCoder) {
    this.canCoder = canCoder;
  }

  @Override
  public void periodic() {
    canCoder.readInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public Angle getPosition() {
    return inputs.absolutePosition;
  }

  public AngularVelocity getAngularVelocity() {
    return inputs.velocity;
  }
}
