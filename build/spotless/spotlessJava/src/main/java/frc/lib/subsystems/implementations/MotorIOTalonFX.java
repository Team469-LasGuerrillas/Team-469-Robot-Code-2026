package frc.lib.subsystems.implementations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystems.configs.ServoMotorSubsystemConfig;
import frc.lib.subsystems.interfaces.MotorIO;

public class MotorIOTalonFX implements MotorIO {
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig config;

  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
  private final MotionMagicVoltage magicalPositionVoltageControl = new MotionMagicVoltage(0);
  private final MotionMagicTorqueCurrentFOC magicalPositionTorqueCurrentFOC =
      new MotionMagicTorqueCurrentFOC(0);
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
}
