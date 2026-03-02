package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
  private static Intake intake = Intake.getInstance();

  public static Command deployAndRun() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngle(Constants.IntakeC.PIVOT_LOWERED),
            () -> intake.setTargetAngle(Constants.IntakeC.PIVOT_LOWERED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(1), () -> intake.setDutyCycle(1)));
  }

  public static Command stow() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngle(Constants.IntakeC.PIVOT_RAISED),
            () -> intake.setTargetAngle(Constants.IntakeC.PIVOT_RAISED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(0), () -> intake.setDutyCycle(0)));
  }

  public static Command agitate() {
    return Commands.repeatingSequence(
        Commands.deadline(Commands.waitSeconds(0.5), stow()),
        Commands.deadline(Commands.waitSeconds(1.2), deployAndRun()));
  }

  public static Command homeAxis() {
    return Commands.none();
  }
}
