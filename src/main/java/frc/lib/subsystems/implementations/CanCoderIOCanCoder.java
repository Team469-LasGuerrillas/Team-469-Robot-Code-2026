package frc.lib.subsystems.implementations;

import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.drivers.CTREUtil;
import frc.lib.subsystems.configs.CanCoderConfig;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;

public class CanCoderIOCanCoder implements CanCoderIO {
  protected final CANcoder canCoder;
  protected CanCoderConfig config;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final BaseStatusSignal[] signals;

  double goodValues = 0.0;

  public CanCoderIOCanCoder(CanCoderConfig config) {
    this.config = config;

    canCoder = new CANcoder(config.CANID.getDeviceNumber(), config.CANID.getBus());

    CTREUtil.applyConfiguration(canCoder, this.config.config);
    positionSignal = canCoder.getAbsolutePosition();
    velocitySignal = canCoder.getVelocity();

    signals = new BaseStatusSignal[] {positionSignal, velocitySignal};

    BaseStatusSignal.setUpdateFrequencyForAll(250.0, signals);

    // Use CANStatusLogger?
  }

  @Override
  public void readInputs(CancoderInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(signals);
    if (Double.isNaN(inputs.absolutePosition.in(Rotation))) {
      BaseStatusSignal.waitForAll(10.0, positionSignal, velocitySignal);
      goodValues++;
    }
    // if (goodValues < 50) return;

    inputs.absolutePosition = positionSignal.getValue();
    inputs.velocity = velocitySignal.getValue();
  }

  @Override
  public void updateFrequency(double hz) {
    BaseStatusSignal.setUpdateFrequencyForAll(hz, signals);
  }
}
