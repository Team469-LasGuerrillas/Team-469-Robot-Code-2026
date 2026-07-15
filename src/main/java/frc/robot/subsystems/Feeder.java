package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Timer;
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

  public boolean unjam = false;

  private int jamLoops = 0;
  private Timer jamTimer = new Timer();

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

  public void setUnjam(boolean unjam) {
    this.unjam = unjam;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    if (unjam) {
      // requestedDutycycle = 0; // Constants.FeederC.REVERSE_DC;
    }
    feeder.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public boolean isRunning() {
    return Math.abs(talonInputs.supplyCurrent.in(Amps)) > 3;
  }

  @Override
  public void periodic() {
    feeder.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);

    if (talonInputs.statorCurrent.in(Amps) > 60
        && talonInputs.motorVelocity.in(RotationsPerSecond) <= 20) {
      jamLoops++;
    } else {
      jamLoops = 0;
    }

    if (jamLoops > 5) {
      jamTimer.start();
      unjam = true;
    }

    if (jamTimer.get() > 0.38) {
      unjam = false;
      jamTimer.reset();
    }
    SignalLogger.writeInteger("FeederJamLoops", jamLoops);
  }
}
