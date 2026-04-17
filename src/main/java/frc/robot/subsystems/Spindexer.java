package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  private static Spindexer instance;

  private final MotorIO floor;
  private final MotorIO secondary;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
  private ServoMotorSubsystemWithFollowersConfig.FollowerConfig followerConfig;

  private double requestedDutycycleFloor = 0;
  private double requestedDutycycleSecondary = 0;

  private boolean unjam = false;

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

  public void setUnjam(boolean unjam) {
    this.unjam = unjam;
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
    if (Feeder.getInstance().unjam) {
      dutyCycle = Constants.SpindexerC.REVERSE_DC;
    }
    setOpenLoopDutyCycleFloor(dutyCycle);
    setOpenLoopDutyCycleSecondary(dutyCycle);
  }

  @Override
  public void periodic() {
    floor.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
