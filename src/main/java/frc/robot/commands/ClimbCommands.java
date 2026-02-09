package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimbCommands {
  public Command climbL1() {
    return Commands.sequence(null);
  }

  public Command climbL3() {
    return Commands.sequence(null);
  }

  public Command setClimbSetpointL1() {
    return Commands.none();
  }

  public Command setClimbSetpointL3() {
    return Commands.none();
  }
}
