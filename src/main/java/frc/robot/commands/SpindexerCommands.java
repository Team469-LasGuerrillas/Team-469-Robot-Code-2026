package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Spindexer;

public class SpindexerCommands {
  private static Spindexer spindexer = Spindexer.getInstance();

  public static Command agitate() {
    return Commands.repeatingSequence(
        Commands.deadline(Commands.waitSeconds(0.05), idleCommand()),
        Commands.deadline(Commands.waitSeconds(0.5), idleReverseCommand()));
  }

  public static Command runPositive() {
    return Commands.startRun(
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.FEEDING_DC),
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.FEEDING_DC),
        spindexer);
  }

  public static Command runReverse() {
    return Commands.startRun(
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.REVERSE_DC),
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.REVERSE_DC),
        spindexer);
  }

  public static Command idleCommand() {
    return Commands.startRun(
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.IDLE_DC),
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.IDLE_DC),
        spindexer);
  }

  public static Command idleReverseCommand() {
    return Commands.startRun(
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.IDLE_REVERSE_DC),
        () -> spindexer.setOpenLoopDutyCycle(Constants.SpindexerC.IDLE_REVERSE_DC),
        spindexer);
  }
}
