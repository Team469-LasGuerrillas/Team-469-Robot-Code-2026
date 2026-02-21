package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;

public class HoodCommands {
  private static final Hood hood = Hood.getInstance();

  public static Command setHoodSetpoint(Angle angle) {
    return Commands.startRun(
        () -> hood.setTargetAngle(angle), () -> hood.setTargetAngle(angle), hood);
  }

  public static Command stowHood() {
    return Commands.startRun(
        () -> hood.setTargetAngle(Constants.HoodC.HOOD_STOW),
        () -> hood.setTargetAngle(Constants.HoodC.HOOD_STOW),
        hood);
  }
}
