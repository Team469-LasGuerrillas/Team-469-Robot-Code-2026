package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ShootTarget;

public class AutonCommands {
  private static Command sweepPath(PathPlannerPath path, boolean rightSide) {
    return Commands.deadline(
        AutoBuilder.followPath(path),
        Commands.sequence(
            Commands.waitSeconds(0.2), Commands.parallel(IntakeCommands.deployAndRun())),
        Commands.sequence(
            Commands.deadline(
                Commands.waitSeconds(0.5),
                FeederCommands.idleCommand(),
                SpindexerCommands.idleCommand())),
        Commands.deferredProxy(
            () ->
                Commands.sequence(
                    Commands.waitSeconds(2.67),
                    // Commands.runOnce(() -> Spindexer.getInstance().setUnjam(true)),
                    Commands.runOnce(() -> Feeder.getInstance().setUnjam(true)),
                    ShooterCommands.targetLaunchSpeed(() -> RotationsPerSecond.of(32)))));
  }

  private static Command pathAndScore(PathPlannerPath path) {
    return Commands.sequence(
        Commands.deadline(
            Drive.getInstance().followPath(path, Constants.DriveC.PP_CONTROLLER_SLOW),
            // Commands.runOnce(() -> Spindexer.getInstance().setUnjam(false)),
            Commands.runOnce(() -> Feeder.getInstance().setUnjam(false)),
            Commands.sequence(
                Commands.deadline(
                    Commands.waitUntil(
                        () ->
                            Drive.getInstance().getPose().getX()
                                    < Constants.Field.BLUE_TRENCH_SCORING.in(Meters) + 0.2
                                || Drive.getInstance().getPose().getX()
                                    > Constants.Field.RED_TRENCH_SCORING.in(Meters) - 0.2),
                    IntakeCommands.deployAndRun()),
                Commands.deadline(
                    Commands.waitSeconds(6),
                    CommandFactory.scoring(),
                    IntakeCommands.deployAndRun()),
                Commands.deadline(
                    Commands.waitSeconds(6), CommandFactory.scoring(), IntakeCommands.agitate()))));
  }

  private static Command depotPathAndScore(PathPlannerPath path) {
    return Commands.deadline(
        Commands.sequence(AutoBuilder.followPath(path), Commands.waitSeconds(6)),
        Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(
                    () ->
                        Drive.getInstance().getPose().getX()
                                < Constants.Field.BLUE_TRENCH_SCORING.in(Meters) + 0.2
                            || Drive.getInstance().getPose().getX()
                                > Constants.Field.RED_TRENCH_SCORING.in(Meters) - 0.2),
                IntakeCommands.deployAndRun()),
            Commands.waitSeconds(0.02),
            Commands.deadline(
                Commands.waitSeconds(2), CommandFactory.scoring(), IntakeCommands.deployAndRun()),
            Commands.deadline(
                Commands.waitSeconds(6), CommandFactory.scoring(), IntakeCommands.agitate())));
  }

  public static Command opStyleAuto(boolean isRed, boolean rightSide) {
    try {

      PathPlannerPath firstSweepPath = PathPlannerPath.fromPathFile("O_A1");
      PathPlannerPath firstScorePath = PathPlannerPath.fromPathFile("O_A2");
      PathPlannerPath secondSweepPath = PathPlannerPath.fromPathFile("O_B1");
      PathPlannerPath secondScorePath = firstScorePath;

      if (rightSide) {
        firstSweepPath = firstSweepPath.mirrorPath();
        firstScorePath = firstScorePath.mirrorPath();
        secondSweepPath = secondSweepPath.mirrorPath();
        secondScorePath = secondScorePath.mirrorPath();
      }

      if (isRed) {
        firstSweepPath = firstSweepPath.flipPath();
        firstScorePath = firstScorePath.flipPath();
        secondSweepPath = secondSweepPath.flipPath();
        secondScorePath = secondScorePath.flipPath();
      }

      return Commands.sequence(
          sweepPath(firstSweepPath, rightSide),
          pathAndScore(firstScorePath),
          Commands.runOnce(() -> ShootTarget.updateNonDynamicGoal(Constants.Field.FIELD_CENTER)),
          sweepPath(secondSweepPath, rightSide),
          pathAndScore(secondScorePath),
          sweepPath(secondSweepPath, rightSide));

    } catch (Exception e) {
      return Commands.run(() -> System.out.println(e.getMessage()));
    }
  }

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
                      Commands.sequence(
                          Commands.waitSeconds(0.05),
                          IntakeCommands.deployAndRun(),
                          FeederCommands.idleCommand(),
                          SpindexerCommands.idleCommand())),
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
                  Commands.deadline(
                      Commands.waitSeconds(3),
                      IntakeCommands.deployAndRun(),
                      FeederCommands.idleCommand(),
                      SpindexerCommands.idleCommand()),
                  Commands.deadline(
                      Commands.waitUntil(
                          () ->
                              Drive.getInstance().getPose().getX()
                                      < Constants.Field.BLUE_TRENCH_SCORING.in(Meters) + 0.5
                                  || Drive.getInstance().getPose().getX()
                                      > Constants.Field.RED_TRENCH_SCORING.in(Meters) - 0.5),
                      IntakeCommands.deployAndRun()),
                  Commands.deadline(
                      Commands.waitSeconds(7), CommandFactory.scoring(), IntakeCommands.agitate())),
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

  public static Command marcusDepotAuto(boolean isRed) {
    try {
      PathPlannerPath thirdPath = PathPlannerPath.fromPathFile("C_D");

      if (isRed) {
        thirdPath = thirdPath.flipPath();
      }

      return Commands.sequence(
          Commands.deadline(AutoBuilder.followPath(thirdPath), IntakeCommands.deployAndRun()),
          Commands.parallel(IntakeCommands.agitate(), CommandFactory.scoring()));

    } catch (Exception e) {
      return Commands.none();
    }
  }

  public static Command depotAuto(boolean isRed, boolean rightSide) {
    try {
      PathPlannerPath thirdPath = PathPlannerPath.fromPathFile("L_B_D");

      if (rightSide) {
        thirdPath = thirdPath.mirrorPath();
      }

      if (isRed) {
        thirdPath = thirdPath.flipPath();
      }

      return Commands.sequence(
          Commands.sequence(
              Commands.deadline(
                  Commands.waitSeconds(2),
                  Commands.sequence(
                      Commands.waitSeconds(0.5),
                      IntakeCommands.agitate(),
                      SpindexerCommands.idleCommand(),
                      FeederCommands.idleCommand())),
              Commands.deadline(Commands.waitSeconds(2), CommandFactory.scoring()),
              AutoBuilder.followPath(thirdPath)),
          Commands.sequence(
              Commands.deadline(
                  Commands.waitSeconds(2),
                  IntakeCommands.deployAndRun(),
                  FeederCommands.idleCommand(),
                  SpindexerCommands.idleCommand()),
              Commands.deadline(
                  Commands.waitSeconds(8), CommandFactory.scoring(), IntakeCommands.agitate())));

    } catch (Exception e) {
      return Commands.none();
    }
  }

  public static Command highAuto(boolean isRed) {

    try {

      PathPlannerPath thirdPathR = PathPlannerPath.fromPathFile("R_M_H");
      PathPlannerPath thirdPathOne = PathPlannerPath.fromPathFile("3_M_H");
      PathPlannerPath thirdPathTwo = PathPlannerPath.fromPathFile("3_M_H2");
      PathPlannerPath thirdPathThree = PathPlannerPath.fromPathFile("3_M_H3");

      if (isRed) {
        thirdPathR = thirdPathR.flipPath();
        thirdPathOne = thirdPathOne.flipPath();
        thirdPathTwo = thirdPathTwo.flipPath();
        thirdPathThree = thirdPathThree.flipPath();
      }

      return Commands.sequence(
          Commands.deadline(
              Commands.waitSeconds(3),
              Drive.getInstance().followPath(thirdPathR, Constants.DriveC.PP_CONTROLLER_SLOW),
              CommandFactory.scoring(),
              IntakeCommands.stow()),
          Commands.deadline(AutoBuilder.followPath(thirdPathOne), IntakeCommands.deployAndRun()),
          Commands.deadline(
              Drive.getInstance().followPath(thirdPathTwo, Constants.DriveC.PP_CONTROLLER_SLOW),
              CommandFactory.scoring(),
              IntakeCommands.agitate()),
          Commands.deadline(
              Commands.waitSeconds(10),
              AutoBuilder.followPath(thirdPathThree),
              IntakeCommands.deployAndRun()));

    } catch (Exception e) {
      return Commands.none();
    }
  }

  public static Command thirdSweepAuto(boolean isRed) {

    try {

      PathPlannerPath thirdPathOne = PathPlannerPath.fromPathFile("3_S_P");
      PathPlannerPath thirdPathTwo = PathPlannerPath.fromPathFile("3_S_P2");

      if (isRed) {
        thirdPathOne = thirdPathOne.flipPath();
      }

      if (isRed) {
        thirdPathTwo = thirdPathTwo.flipPath();
      }

      return Commands.sequence(
          Commands.deadline(
              Commands.waitSeconds(2.5), CommandFactory.scoring(), IntakeCommands.stow()),
          Commands.deadline(AutoBuilder.followPath(thirdPathOne), IntakeCommands.deployAndRun()),
          Commands.deadline(
              Drive.getInstance().followPath(thirdPathTwo, Constants.DriveC.PP_CONTROLLER_SLOW),
              CommandFactory.scoring(),
              IntakeCommands.deployAndRun()),
          Commands.deadline(
              Commands.waitSeconds(10),
              CommandFactory.scoring(),
              Commands.sequence(
                  Commands.deadline(Commands.waitSeconds(2), IntakeCommands.deployAndRun()),
                  IntakeCommands.stow())));

    } catch (Exception e) {
      return Commands.none();
    }
  }

  public static Command feedAuto(boolean isRed, boolean rightSide) {

    try {

      PathPlannerPath ahh67 = PathPlannerPath.fromPathFile("F_P");

      if (isRed) {
        ahh67 = ahh67.flipPath();
      }

      if (rightSide) {
        ahh67 = ahh67.mirrorPath();
      }

      return Commands.sequence(
          Commands.deadline(
              Commands.waitSeconds(3.3), CommandFactory.feedOrScore(), IntakeCommands.stow()),
          Commands.deadline(
              AutoBuilder.followPath(ahh67),
              Commands.parallel(IntakeCommands.deployAndRun(), CommandFactory.feedOrScore())),
          Commands.deadline(
              Commands.waitSeconds(8), CommandFactory.feedOrScore(), IntakeCommands.agitate()));
    } catch (Exception e) {
      return Commands.none();
    }
  }

  private static Command faultResistantPathFollow(PathPlannerPath path) {
    Pose2d endPoint = path.getPathPoses().get(path.getPathPoses().size() - 1);

    return Commands.race(
        Commands.race(
            Commands.sequence(
                Commands.waitUntil(
                    () ->
                        (RobotState.getTrajectoryTarget()
                                .getTranslation()
                                .getDistance(Drive.getInstance().getPose().getTranslation()))
                            > 1),
                Commands.waitSeconds(4.414)),
            AutoBuilder.followPath(path)),
        Commands.sequence(
            Commands.waitUntil(
                () ->
                    (RobotState.getTrajectoryTarget()
                            .getTranslation()
                            .getDistance(Drive.getInstance().getPose().getTranslation()))
                        > 1),
            AutoBuilder.pathfindToPose(endPoint, Constants.DriveC.defaultConstraints)));
  }
}
