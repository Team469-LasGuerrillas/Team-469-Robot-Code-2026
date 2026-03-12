package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AutonCommands {
  public static Command leftPassRed() {
    try {
      return Commands.sequence(
          Commands.deadline(
              Commands.sequence(
                  Commands.deadline(Commands.waitSeconds(3), IntakeCommands.deployAndRun()),
                  Commands.deadline(
                      Commands.waitUntil(
                          () ->
                              Drive.getInstance().getPose().getX()
                                      < Constants.Field.BLUE_TRENCH_SCORING.in(Meters) + 0.5
                                  || Drive.getInstance().getPose().getX()
                                      > Constants.Field.RED_TRENCH_SCORING.in(Meters) - 0.5),
                      IntakeCommands.deployAndRun()),
                  Commands.deadline(
                      Commands.waitSeconds(5), CommandFactory.scoring(), IntakeCommands.agitate())),
              AutoBuilder.followPath(PathPlannerPath.fromPathFile("S_A").flipPath())),
          Commands.deadline(
              Commands.sequence(
                  Commands.deadline(Commands.waitSeconds(3), IntakeCommands.deployAndRun()),
                  Commands.deadline(
                      Commands.waitUntil(
                          () ->
                              Drive.getInstance().getPose().getX()
                                      < Constants.Field.BLUE_TRENCH_SCORING.in(Meters) + 0.5
                                  || Drive.getInstance().getPose().getX()
                                      > Constants.Field.RED_TRENCH_SCORING.in(Meters) - 0.5),
                      IntakeCommands.deployAndRun()),
                  Commands.deadline(
                      Commands.waitSeconds(5), CommandFactory.scoring(), IntakeCommands.agitate())),
              AutoBuilder.followPath(PathPlannerPath.fromPathFile("S_B").flipPath())));
    } catch (Exception e) {
      return Commands.none();
    }
  }
}
