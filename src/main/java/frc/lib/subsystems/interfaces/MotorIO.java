package frc.lib.subsystems.interfaces;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.drivers.CANDeviceId;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  public class MotorInputs {
    public Angle motorPosition = Rotations.of(0);
    public Angle rawMotorPosition = Rotations.of(0);
    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration motorAngularAcceleration = RotationsPerSecondPerSecond.of(0);

    public Angle targetPosition = Rotations.of(0);
    public AngularVelocity targetVelocity = RotationsPerSecond.of(0);

    public Voltage appliedVolts = Volts.of(0);
    public Current supplyCurrent = Amps.of(0);
    public Current statorCurrent = Amps.of(0);
  }

  default Angle clampPosition(Angle units) {
    return null;
  }

  default Angle wrapPosition(Angle units) {
    return null;
  }

  default void readInputs(MotorInputsAutoLogged inputs) {}

  default void setOpenLoopDutyCycle(double dutyCycle) {}

  default void setPositionSetpoint(Angle position, double ff) {}

  default void setMagicalPositionSetpoint(
      Angle position,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      int slot,
      double ff) {}

  default void setVelocitySetpiont(AngularVelocity velocity) {}

  default void setMagicalVelcoitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {}

  default void setEnableSoftLimits(boolean forward, boolean reverse) {}

  default void setNeutralMode(NeutralModeValue mode) {}

  default void setMotionMagicConfig(MotionMagicConfigs config) {}

  default void setZero() {
    setCurrentPosition(Rotations.of(0));
  }

  default void setCurrentPosition(Angle position) {}

  default void follow(CANDeviceId canId, boolean opposeLeader) {}
}
