package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;

public class FeederCommands {
  private static Feeder feeder = Feeder.getInstance();

  public static Command runPositive() {
    return Commands.startRun(
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.FEEDING_DC),
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.FEEDING_DC),
        feeder);
  }

  public static Command runReverse() {
    return Commands.startRun(
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.REVERSE_DC),
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.REVERSE_DC),
        feeder);
  }

  public static Command idleReverse() {
    return Commands.startRun(
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.IDLE_REVERSE_DC),
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.IDLE_REVERSE_DC),
        feeder);
  }

  public static Command idleCommand() {
    return Commands.startRun(
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.IDLE_DC),
        () -> feeder.setOpenLoopDutyCycle(Constants.FeederC.IDLE_DC),
        feeder);
  }

  public static Command retract() {
    return Commands.deadline(Commands.waitSeconds(0.2), runReverse());
  }
}
