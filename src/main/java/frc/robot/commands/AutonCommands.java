package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AutonCommands {
  public static Command leftPass() {
    try {
      return Commands.sequence(
          Commands.deadline(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile("L_A").flipPath()),
              IntakeCommands.deployAndRun()),
          Commands.deadline(
              Commands.waitSeconds(5), CommandFactory.scoring(), IntakeCommands.agitate()),
          Commands.deadline(
              AutoBuilder.followPath(PathPlannerPath.fromPathFile("L_B").flipPath()),
              IntakeCommands.deployAndRun()),
          Commands.deadline(
              Commands.waitSeconds(5), CommandFactory.scoring(), IntakeCommands.agitate()));
    } catch (Exception e) {
      return Commands.none();
    }
  }
}
