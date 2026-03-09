package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

public class ClimbCommands {
  private static Climb climb = Climb.GetInstance();

  public static Command climbL1() {
    return Commands.sequence(null);
  }

  public static Command setClimbSetpointStow() {
    return Commands.startRun(
        () -> climb.setPosition(Constants.ClimbC.CLIMB_STOW),
        () -> climb.setPosition(Constants.ClimbC.CLIMB_STOW),
        climb);
  }

  public static Command setClimbSetpointClear() {
    return Commands.startRun(
        () -> climb.setPosition(Constants.ClimbC.CLIMB_CLEAR),
        () -> climb.setPosition(Constants.ClimbC.CLIMB_CLEAR),
        climb);
  }

  public static Command setClimbSetpointRaise() {
    return Commands.startRun(
        () -> climb.setPosition(Constants.ClimbC.CLIMB_STOW),
        () -> climb.setPosition(Constants.ClimbC.CLIMB_STOW),
        climb);
  }

  public static Command setClimbOff() {
    return Commands.startRun(() -> climb.setDutyCycle(0), () -> climb.setDutyCycle(0), climb);
  }

  public static Command setClimbUp() {
    return Commands.startRun(() -> climb.setDutyCycle(0.7), () -> climb.setDutyCycle(0.7), climb);
  }

  public static Command setClimbDown() {
    return Commands.startRun(() -> climb.setDutyCycle(-1), () -> climb.setDutyCycle(-1), climb);
  }
}
