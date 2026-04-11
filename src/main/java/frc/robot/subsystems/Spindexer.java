package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;

public class Spindexer extends SubsystemBase {
  private static Spindexer instance;

  private final MotorIO lead;
  private final MotorIO follower;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
  private ServoMotorSubsystemWithFollowersConfig.FollowerConfig followerConfig;

  private double requestedDutycycle = 0;

  public static Spindexer createinstance(MotorIO floorMotorIo, MotorIO secondaryMotorIo) {
    instance = new Spindexer(floorMotorIo, secondaryMotorIo);
    return instance;
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private Spindexer(MotorIO floor, MotorIO secondary) {
    this.lead = floor;
    this.follower = secondary;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    lead.setOpenLoopDutyCycle(requestedDutycycle);
  }

  @Override
  public void periodic() {
    lead.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
