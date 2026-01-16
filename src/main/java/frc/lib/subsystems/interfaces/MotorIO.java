package frc.lib.subsystems.interfaces;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  @AutoLog
  public class MotorInputs {
    public Angle motorPosition = Rotations.of(0);
    public Angle rawMotorPosition = Rotations.of(0);
    public AngularVelocity motorVelocity = RotationsPerSecond.of(0);
    public AngularAcceleration motorAngularAcceleration = RotationsPerSecondPerSecond.of(0);

    public Angle targetPosition = Angle.ofBaseUnits(0, null);
    public AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(0, null);

    public Voltage appliedVolts = Voltage.ofBaseUnits(0, null);
    public Current supplyCurrent = Current.ofBaseUnits(0, null);
    public Current statorCurrent = Current.ofBaseUnits(0, null);
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
}
