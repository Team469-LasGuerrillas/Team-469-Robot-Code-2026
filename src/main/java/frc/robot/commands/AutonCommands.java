package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.AutonPaths;
import frc.robot.subsystems.drive.Drive;

public class AutonCommands {
  public static Command redPass() {
    return Commands.sequence(
        Commands.deadline(Commands.waitSeconds(3), CommandFactory.scoring()),
        Commands.parallel(
            Commands.deferredProxy(
                () ->
                    Drive.getInstance()
                        .followPath(AutonPaths.leftAutoRed(Drive.getInstance().getPose()))),
            IntakeCommands.deployAndRun()));
  }
}
