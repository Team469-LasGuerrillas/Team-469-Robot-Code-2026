package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Spindexer extends SubsystemBase {
  private static Spindexer instance;

  private final MotorIO spindexer;
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();

  private final CanCoderIO canCoder;

  private double requestedDutycycle = 0;

  public Spindexer createinstance() {
    return instance;
  }

  public static Spindexer Getinstance() {
    return instance;
  }

  private Spindexer(MotorIO spindexer, CanCoderIO canCoder) {
    this.spindexer = spindexer;
    this.canCoder = canCoder;

    spindexer.setEnableSoftLimits(true, true);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    spindexer.setOpenLoopDutyCycle(requestedDutycycle);
  }

  @Override
  public void periodic() {
    spindexer.readInputs(talonInputs);
    Logger.processInputs(getName() + "Motor", talonInputs);
  }
}
