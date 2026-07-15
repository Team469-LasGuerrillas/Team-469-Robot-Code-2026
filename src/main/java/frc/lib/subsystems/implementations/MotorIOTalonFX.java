package frc.lib.subsystems.implementations;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.drivers.CANDeviceId;
import frc.lib.drivers.CANStatusLogger;
import frc.lib.drivers.CTREUtil;
import frc.lib.subsystems.configs.ServoMotorSubsystemConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import java.util.ArrayList;

public class MotorIOTalonFX implements MotorIO {
  protected final TalonFX talon;
  protected final ServoMotorSubsystemConfig config;

  protected final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC = new PositionTorqueCurrentFOC(0);
  private final DynamicMotionMagicVoltage magicalPositionVoltageControl =
      new DynamicMotionMagicVoltage(0, 0, 0);
  private final DynamicMotionMagicTorqueCurrentFOC magicalPositionTorqueCurrentFOC =
      new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0);
  private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
  private final MotionMagicVelocityVoltage magicalVelocityVoltgaeControl =
      new MotionMagicVelocityVoltage(0);
  private final MotionMagicVelocityTorqueCurrentFOC magicalVelocityTorqueCurrentFOC =
      new MotionMagicVelocityTorqueCurrentFOC(0);
  private final Follower followerControl = new Follower(0, MotorAlignmentValue.Aligned);

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  private final StatusSignal<Angle> rawRotorPositionSignal;

  private final StatusSignal<Double> targetPositionSignal;

  private final StatusSignal<Current> torqueCurrentSignal;

  private final BaseStatusSignal[] signals;

  private static final ArrayList<BaseStatusSignal> canivoreASignals = new ArrayList<>();
  private static final ArrayList<BaseStatusSignal> canivoreBSignals = new ArrayList<>();

  boolean autoLimit = false;

  public MotorIOTalonFX(ServoMotorSubsystemConfig config) {
    this.config = config;
    talon = new TalonFX(config.talonCANID.getDeviceNumber(), config.talonCANID.getBus());

    CTREUtil.applyConfiguration(talon, this.config.fxConfig);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    accelSignal = talon.getAcceleration();
    voltageSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();
    rawRotorPositionSignal = talon.getRotorPosition();

    targetPositionSignal = talon.getClosedLoopReference();

    torqueCurrentSignal = talon.getTorqueCurrent();

    signals =
        new BaseStatusSignal[] {
          positionSignal,
          velocitySignal,
          voltageSignal,
          currentStatorSignal,
          currentSupplySignal,
          rawRotorPositionSignal,
          targetPositionSignal
        };

    CANStatusLogger.getInstance().registerTalonFX(config.name, talon, config.talonCANID);

    if (talon.getDeviceID() == 14 || talon.getDeviceID() == 12) {
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(1000, torqueCurrentSignal),
          talon.getDeviceID());
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(250, signals), talon.getDeviceID());
    } else if (talon.getDeviceID() == 20) {
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(500, signals), talon.getDeviceID());
    } else {
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(50, torqueCurrentSignal),
          talon.getDeviceID());
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(250, signals), talon.getDeviceID());
    }
    CTREUtil.tryUntilOK(() -> talon.optimizeBusUtilization(), talon.getDeviceID());

    if (config.talonCANID.getBus().equals("469CanivoreB")) {
      for (BaseStatusSignal s : signals) {
        canivoreBSignals.add(s);
      }
    } else {
      for (BaseStatusSignal s : signals) {
        canivoreASignals.add(s);
      }
    }
  }

  private double rotorToUnits(double rotor) {
    return rotor * config.unitToRotorRatio;
  }

  @Override
  public Angle clampPosition(Angle units) {
    return Rotations.of(
        unitsToRotor(
            MathUtil.clamp(
                units.in(Rotations), config.kMinPositionUnits, config.kMaxPositionUnits)));
  }

  @Override
  public Angle wrapPosition(Angle units) {
    return Rotations.of(unitsToRotor(((units.in(Rotations) + 0.5) % 1) - 0.5));
  }

  public double unitsToRotor(double units) {
    return units / config.unitToRotorRatio;
  }

  public static void refreshAllSignals() {
    BaseStatusSignal.refreshAll(canivoreASignals);
    BaseStatusSignal.refreshAll(canivoreBSignals);
  }

  @Override
  public void readInputs(MotorInputsAutoLogged inputs) {
    // BaseStatusSignal.waitForAll(0.0, signals);
    inputs.motorPosition = Rotations.of(rotorToUnits(positionSignal.getValueAsDouble()));
    inputs.motorVelocity = RotationsPerSecond.of(rotorToUnits(velocitySignal.getValueAsDouble()));
    inputs.motorAngularAcceleration =
        RotationsPerSecondPerSecond.of(rotorToUnits(accelSignal.getValueAsDouble()));
    inputs.targetPosition = Rotations.of(rotorToUnits(targetPositionSignal.getValueAsDouble()));
    inputs.targetVelocity =
        RotationsPerSecond.of(rotorToUnits(targetPositionSignal.getValueAsDouble()));

    inputs.appliedVolts = Volts.of(voltageSignal.getValueAsDouble());
    inputs.supplyCurrent = Amps.of(currentSupplySignal.getValueAsDouble());
    inputs.statorCurrent = Amps.of(currentStatorSignal.getValueAsDouble());
    inputs.rawMotorPosition = Revolutions.of(rawRotorPositionSignal.getValueAsDouble());

    // if (talon.getDeviceID() == 10 && DriverStation.isAutonomous()) {
    //   // This is auto
    //   if (autoLimit == false) {
    //     CurrentLimitsConfigs newConfig =
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(150)
    //             .withStatorCurrentLimitEnable(true)
    //             .withSupplyCurrentLimit(45)
    //             .withSupplyCurrentLimitEnable(true)
    //             .withSupplyCurrentLowerLimit(30)
    //             .withSupplyCurrentLowerTime(3);
    //     talon.getConfigurator().apply(newConfig, 0);
    //     autoLimit = true;
    //   }
    // } else if (talon.getDeviceID() == 10) {
    //   if (autoLimit = true) {
    //     CurrentLimitsConfigs newConfig =
    //         new CurrentLimitsConfigs()
    //             .withStatorCurrentLimit(150)
    //             .withStatorCurrentLimitEnable(true)
    //             .withSupplyCurrentLimit(45)
    //             .withSupplyCurrentLimitEnable(true)
    //             .withSupplyCurrentLowerLimit(30)
    //             .withSupplyCurrentLowerTime(0.5);
    //     talon.getConfigurator().apply(newConfig, 0);
    //     autoLimit = false;
    //   }
    // }
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(dutyCycleControl.withOutput(dutyCycle).withEnableFOC(true));
  }

  @Override
  public void setPositionSetpoint(Angle position, AngularVelocity velocity, double ff) {
    if (config.outputMode == ClosedLoopOutputType.TorqueCurrentFOC) {
      talon.setControl(
          positionTorqueCurrentFOC
              .withPosition(position)
              .withVelocity(velocity)
              .withFeedForward(ff));
    } else {
      talon.setControl(
          positionVoltageControl
              .withPosition(position)
              .withVelocity(velocity)
              .withFeedForward(ff)
              .withEnableFOC(true));
    }
  }

  @Override
  public void setMagicalPositionSetpoint(
      Angle position,
      AngularVelocity velocity,
      AngularAcceleration acceleration,
      int slot,
      double ff) {
    if (config.outputMode == ClosedLoopOutputType.TorqueCurrentFOC) {
      talon.setControl(
          magicalPositionTorqueCurrentFOC
              .withPosition(position)
              .withVelocity(velocity)
              .withAcceleration(acceleration)
              .withFeedForward(ff));
    } else {
      talon.setControl(
          magicalPositionVoltageControl
              .withPosition(position)
              .withVelocity(velocity)
              .withAcceleration(acceleration)
              .withFeedForward(ff)
              .withEnableFOC(true));
    }
  }

  @Override
  public void setVelocitySetpiont(AngularVelocity velocity) {
    if (config.outputMode == ClosedLoopOutputType.TorqueCurrentFOC) {
      if (Math.abs(velocity.in(RotationsPerSecond)) < 0.00001) {
        talon.setControl(
            velocityTorqueCurrentFOC
                .withVelocity(unitsToRotor(velocity.in(RotationsPerSecond)))
                .withSlot(1));
      } else {
        talon.setControl(
            velocityTorqueCurrentFOC
                .withVelocity(unitsToRotor(velocity.in(RotationsPerSecond)))
                .withSlot(0));
      }

    } else {
      talon.setControl(
          velocityVoltageControl
              .withVelocity(unitsToRotor(velocity.in(RotationsPerSecond)))
              .withEnableFOC(true));
    }
  }

  @Override
  public void setMagicalVelcoitySetpoint(
      AngularVelocity velocity, AngularAcceleration acceleration, int slot) {
    if (config.outputMode == ClosedLoopOutputType.TorqueCurrentFOC) {
      talon.setControl(
          magicalVelocityTorqueCurrentFOC
              .withVelocity(unitsToRotor(velocity.in(RotationsPerSecond)))
              .withAcceleration(acceleration));
    } else {
      talon.setControl(
          magicalVelocityVoltgaeControl
              .withVelocity(unitsToRotor(velocity.in(RotationsPerSecond)))
              .withAcceleration(acceleration)
              .withEnableFOC(true));
    }
  }

  @Override
  public void setEnableSoftLimits(boolean fwd, boolean rev) {
    config.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = fwd;
    config.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = rev;
    CTREUtil.applyConfiguration(talon, config.fxConfig);
  }

  @Override
  public void setNeutralMode(NeutralModeValue mode) {
    talon.setNeutralMode(mode);
  }

  @Override
  public void setCurrentPosition(Angle position) {
    talon.setPosition(unitsToRotor(position.in(Rotations)));
  }

  @Override
  public void follow(CANDeviceId masterId, boolean opposeLeader) {
    MotorAlignmentValue alignmentValue;

    if (opposeLeader) {
      alignmentValue = MotorAlignmentValue.Opposed;
    } else {
      alignmentValue = MotorAlignmentValue.Aligned;
    }

    CTREUtil.tryUntilOK(
        () ->
            talon.setControl(
                followerControl
                    .withLeaderID(masterId.getDeviceNumber())
                    .withMotorAlignment(alignmentValue)),
        this.config.talonCANID.getDeviceNumber());
  }
}
