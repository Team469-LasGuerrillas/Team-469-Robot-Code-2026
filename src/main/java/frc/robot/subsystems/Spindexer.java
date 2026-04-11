package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;

public class Spindexer extends SubsystemBase {
  private static Spindexer instance;

  private final MotorIO floor;
  private final MotorIO secondary;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
  private ServoMotorSubsystemWithFollowersConfig.FollowerConfig followerConfig;

  private double requestedDutycycleFloor = 0;
  private double requestedDutycycleSecondary = 0;

  public static Spindexer createinstance(MotorIO floorMotorIo, MotorIO secondaryMotorIo) {
    instance = new Spindexer(floorMotorIo, secondaryMotorIo);
    return instance;
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private Spindexer(MotorIO floor, MotorIO secondary) {
    this.floor = floor;
    this.secondary = secondary;
  }

  public void setOpenLoopDutyCycleFloor(double dutyCycle) {
    requestedDutycycleFloor = dutyCycle;
    floor.setOpenLoopDutyCycle(requestedDutycycleFloor);
  }

  public void setOpenLoopDutyCycleSecondary(double dutyCycle) {
    requestedDutycycleSecondary = dutyCycle;
    secondary.setOpenLoopDutyCycle(requestedDutycycleSecondary);
  }

  public void setOpenLoopDutyCycleBoth(double dutyCycle) {
    setOpenLoopDutyCycleFloor(dutyCycle);
    setOpenLoopDutyCycleSecondary(dutyCycle);
  }

  @Override
  public void periodic() {
    floor.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
