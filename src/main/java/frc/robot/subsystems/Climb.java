package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private static Climb instance;

  private final MotorIO climbMotor;
  private final MotorIO climbFollowerMotor;
  private final MotorInputsAutoLogged climbInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged climbFollowerInputs = new MotorInputsAutoLogged();

  private double requestedDutyCycle = 0;

  public static Climb createInstance(MotorIO climbMotor, MotorIO climbFollowerMotor) {
    instance = new Climb(climbMotor, climbFollowerMotor);
    return instance;
  }

  public static Climb GetInstance() {
    return instance;
  }

  private Climb(MotorIO climbMotor, MotorIO climbFollowerMotor) {
    this.climbMotor = climbMotor;
    this.climbFollowerMotor = climbFollowerMotor;

    climbMotor.setEnableSoftLimits(true, true);
    climbFollowerMotor.setEnableSoftLimits(true, true);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutyCycle = dutyCycle;
    climbMotor.setOpenLoopDutyCycle(dutyCycle);
    climbFollowerMotor.setOpenLoopDutyCycle(dutyCycle);
  }

  @Override
  public void periodic() {
    climbMotor.readInputs(climbInputs);
    Logger.processInputs(getName() + "Climb Motor", climbInputs);
    climbFollowerMotor.readInputs(climbInputs);
    Logger.processInputs(getName() + "Climb Follower Motor", climbFollowerInputs);
  }
}
