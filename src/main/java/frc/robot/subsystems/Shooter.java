package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.lib.utilities.math.ToleranceUtil;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.RobotState.FlywheelState;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private static Shooter instance;

  private final MotorIO leadFlywheel;

  private final MotorInputsAutoLogged leadInputs = new MotorInputsAutoLogged();

  private ServoMotorSubsystemWithFollowersConfig.FollowerConfig[] followerConfigs;

  private double requestedDutycycle = 0;
  private AngularVelocity requestedAngularVelocity = RotationsPerSecond.of(0);

  public static Shooter createinstance(
      ServoMotorSubsystemWithFollowersConfig leadConfig, MotorIO leadIo, MotorIO[] followerIo) {
    instance = new Shooter(leadConfig, leadIo, followerIo);
    return instance;
  }

  public static Shooter getInstance() {
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

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    leadFlywheel.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public void setWatermarkTorqueCurrentFOC(AngularVelocity velocity) {
    requestedAngularVelocity = velocity;
    leadFlywheel.setVelocitySetpiont(velocity);
  }

  @Override
  public void periodic() {
    leadFlywheel.readInputs(leadInputs);
    Logger.processInputs(getName() + "flywheelLead", leadInputs);

    boolean onTargetHub =
        ToleranceUtil.epsilonEquals(
            getSpeed().in(RotationsPerSecond),
            requestedAngularVelocity.in(RotationsPerSecond),
            Constants.LauncherC.HUB_SPEED_TOLERANCE.in(RotationsPerSecond));

    boolean onTargetPass =
        ToleranceUtil.epsilonEquals(
            getSpeed().in(RotationsPerSecond),
            requestedAngularVelocity.in(RotationsPerSecond),
            Constants.LauncherC.PASS_SPEED_TOLERANCE.in(RotationsPerSecond));

    if (onTargetHub) {
      RobotState.setFlywheelState(FlywheelState.HUBLOCKED);
    } else if (onTargetPass) {
      RobotState.setFlywheelState(FlywheelState.PASSLOCKED);
    } else {
      RobotState.setFlywheelState(FlywheelState.UNLOCKED);
    }
  }

  public AngularVelocity getSpeed() {
    return leadInputs.motorVelocity;
  }
}
