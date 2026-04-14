package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;

public class Feeder extends SubsystemBase {
  private static Feeder instance;

  private final MotorIO feeder;
  private final MotorIO follower;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  public static Feeder createinstance(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO feeder, MotorIO follower) {
    instance = new Feeder(leadConfig, feeder, follower);
    return instance;
  }

  public static Feeder getInstance() {
    return instance;
  }

  private Feeder(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO feeder, MotorIO follower) {
    this.feeder = feeder;
    this.follower = follower;

    follower.follow(leadConfig.talonCANID, leadConfig.followers[0].inverted);
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    feeder.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public boolean isRunning() {
    return Math.abs(talonInputs.supplyCurrent.in(Amps)) > 5;
  }

  @Override
  public void periodic() {
    feeder.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
