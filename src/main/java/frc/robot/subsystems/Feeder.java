package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private static Feeder instance;

  private final MotorIO feeder;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  public static Feeder createinstance(MotorIO feeder) {
    instance = new Feeder(feeder);
    return instance;
  }

  public static Feeder getInstance() {
    return instance;
  }

  private Feeder(MotorIO feeder) {
    this.feeder = feeder;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    System.out.println(dutyCycle);
    feeder.setOpenLoopDutyCycle(requestedDutycycle);
  }

  @Override
  public void periodic() {
    feeder.readInputs(talonInputs);
    Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
