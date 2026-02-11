package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static Shooter instance;

  private final MotorIO leadFlywheel;

  private final MotorInputsAutoLogged leadInputs = new MotorInputsAutoLogged();

  private ServoMotorSubsystemWithFollowersConfig.FollowerConfig[] followerConfigs;

  private double requestedDutycycle = 0;

  public Shooter createinstance(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO leadIo, MotorIO[] followerIo) {
    instance = new Shooter(leadConfig, leadIo, followerIo);
    return instance;
  }

  public static Shooter Getinstance() {
    return instance;
  }

  private Shooter(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO leadIo, MotorIO[] followerIo) {
    this.followerConfigs = leadConfig.followers;

    for (int i = 0; i < followerConfigs.length; i++) {
      MotorIO followerIO = followerIo[i];
      followerIO.follow(leadConfig.talonCANID, followerConfigs[i].inverted);
      ;
    }

    this.leadFlywheel = leadIo;
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    leadFlywheel.setOpenLoopDutyCycle(requestedDutycycle);
  }

  @Override
  public void periodic() {
    leadFlywheel.readInputs(leadInputs);
    Logger.processInputs(getName() + "flywheelLead", leadInputs);
  }
}
