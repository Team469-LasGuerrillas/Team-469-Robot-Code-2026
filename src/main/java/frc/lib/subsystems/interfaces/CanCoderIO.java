package frc.lib.subsystems.interfaces;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface CanCoderIO {

  @AutoLog
  public class CancoderInputs {
    public Angle absolutePosition = Rotations.of(0);
    public AngularVelocity velocity = RotationsPerSecond.of(0);
  }

  default void readInputs(CancoderInputsAutoLogged inputs) {}

  default void updateFrequency(double hz) {}
}
