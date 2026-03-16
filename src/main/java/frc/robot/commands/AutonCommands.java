package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class AutonCommands {
  public static Command leftPassRed(boolean isRed, boolean rightSide) {
    try {

      PathPlannerPath firstPath = PathPlannerPath.fromPathFile("S_A");
      PathPlannerPath secondPath = PathPlannerPath.fromPathFile("S_C");

      if (rightSide) {
        firstPath = firstPath.mirrorPath();
        secondPath = secondPath.mirrorPath();
      }

      if (isRed) {
        firstPath = firstPath.flipPath();
        secondPath = secondPath.flipPath();
      }

      return Commands.sequence(
          Commands.deadline(
              Commands.sequence(
                  Commands.deadline(
                      Commands.waitSeconds(3),
                      Commands.sequence(Commands.waitSeconds(0.3), IntakeCommands.deployAndRun())),
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
              AutoBuilder.followPath(firstPath)),
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
                      Commands.waitSeconds(6), CommandFactory.scoring(), IntakeCommands.agitate())),
              AutoBuilder.followPath(secondPath)));
    } catch (Exception e) {
      return Commands.none();
    }
  }

  public static Command testAuto() {
    try {
      return AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestA").flipPath());
    } catch (Exception e) {
      return Commands.none();
    }
  }
}
