package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Hood;
import java.util.function.Supplier;

public class HoodCommands {
  private static final Hood hood = Hood.getInstance();

  public static Command setHoodSetpoint(Supplier<Angle> angle) {
    return Commands.startRun(
        () -> hood.setTargetAngle(angle.get()), () -> hood.setTargetAngle(angle.get()), hood);
  }

  public static Command stowHood() {
    return Commands.startRun(
        () -> hood.setTargetAngle(Constants.HoodC.HOOD_STOW),
        () -> hood.setTargetAngle(Constants.HoodC.HOOD_STOW),
        hood);
  }
}
