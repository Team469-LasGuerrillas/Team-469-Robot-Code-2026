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

  public static Spindexer createinstance(
      ServoMotorSubsystemWithFollowersConfig leadConfig,
      MotorIO leadMotorIo,
      MotorIO followerMotorIo) {
    instance = new Spindexer(leadConfig, leadMotorIo, followerMotorIo);
    return instance;
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private Spindexer(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO lead, MotorIO follower) {
    this.lead = lead;
    this.follower = follower;
    this.follower.follow(leadConfig.talonCANID, leadConfig.followers[0].inverted);
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
