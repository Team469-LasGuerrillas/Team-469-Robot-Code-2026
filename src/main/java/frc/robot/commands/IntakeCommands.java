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
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(1), () -> intake.setDutyCycle(1)));
  }

  public static Command deployAndDontRun() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(0), () -> intake.setDutyCycle(0)));
  }

  public static Command deployAndRunReverse() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_LOWERED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(-1), () -> intake.setDutyCycle(-1)));
  }

  public static Command stow() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngleSlow(Constants.IntakeC.PIVOT_RAISED),
            () -> intake.setTargetAngleSlow(Constants.IntakeC.PIVOT_RAISED),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(0.3), () -> intake.setDutyCycle(0)));
  }

  public static Command agitateSlow() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngleSlow(Constants.IntakeC.PIVOT_AGITATE),
            () -> intake.setTargetAngleSlow(Constants.IntakeC.PIVOT_AGITATE),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(0.3), () -> intake.setDutyCycle(0)));
  }

  public static Command pivotToAgitate() {
    return Commands.parallel(
        Commands.startRun(
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_AGITATE),
            () -> intake.setTargetAngleFast(Constants.IntakeC.PIVOT_AGITATE),
            intake),
        Commands.startRun(() -> intake.setDutyCycle(0), () -> intake.setDutyCycle(0.2)));
  }

  public static Command agitate() {
    return Commands.repeatingSequence(
        Commands.deadline(Commands.waitSeconds(0.3), pivotToAgitate()),
        Commands.deadline(Commands.waitSeconds(0.7), deployAndRun()));
  }

  public static Command agitateThenStow() {
    return Commands.sequence(
        Commands.deadline(Commands.waitSeconds(0.3), agitateSlow()),
        Commands.waitSeconds(0.5),
        Commands.deadline(Commands.waitSeconds(0.8), agitateSlow()),
        Commands.deadline(Commands.waitSeconds(2), agitate()),
        stow());
  }

  public static Command homeAxis() {
    return Commands.none();
  }
}
