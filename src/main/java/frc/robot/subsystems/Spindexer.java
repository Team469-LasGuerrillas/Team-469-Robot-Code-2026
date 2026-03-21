package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;

public class Spindexer extends SubsystemBase {
  private static Spindexer instance;

  private final MotorIO spindexer;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  public static Spindexer createinstance(MotorIO spindexer) {
    instance = new Spindexer(spindexer);
    return instance;
  }

  public static Spindexer getInstance() {
    return instance;
  }

  private Spindexer(MotorIO spindexer) {
    this.spindexer = spindexer;
  }

  public void setOpenLoopDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    spindexer.setOpenLoopDutyCycle(requestedDutycycle);
  }

  @Override
  public void periodic() {
    spindexer.readInputs(talonInputs);
    // Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
