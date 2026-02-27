package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.utilities.field.Station;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.ShootTarget;

public class CommandFactory {
  public static Command passing() {
    return Commands.parallel(
        Commands.either(
            Commands.startRun(
                () -> ShootTarget.updateGoal(Constants.Field.RED_HUB, true),
                () -> ShootTarget.updateGoal(Constants.Field.RED_HUB, true)),
            Commands.startRun(
                () -> ShootTarget.updateGoal(Constants.Field.BLUE_HUB, true),
                () -> ShootTarget.updateGoal(Constants.Field.BLUE_HUB, true)),
            () -> Station.isRed()),
        TurretCommands.targetPoint(ShootTarget.getTranslationToTarget()),
        HoodCommands.setHoodSetpoint(
            Degrees.of(
                Constants.LauncherC.SHOOTER_HOOD_MAP_PASSING.get(
                    ShootTarget.getDistanceToTarget().in(Meters)))),
        ShooterCommands.targetLaunchSpeed(
            RotationsPerSecond.of(
                Constants.LauncherC.FLYWHEEL_SHOT_SPEEDMAP_PASSING.get(
                    ShootTarget.getDistanceToTarget().in(Meters)))),
        feedWhenReadyPass());
  }

  public static Command scoring() {
    return Commands.parallel(
        Commands.either(
            Commands.startRun(
                () -> ShootTarget.updateGoal(Constants.Field.RED_HUB, false),
                () -> ShootTarget.updateGoal(Constants.Field.RED_HUB, false)),
            Commands.startRun(
                () -> ShootTarget.updateGoal(Constants.Field.BLUE_HUB, false),
                () -> ShootTarget.updateGoal(Constants.Field.BLUE_HUB, false)),
            () -> Station.isRed()),
        TurretCommands.targetPoint(ShootTarget.getTranslationToTarget()),
        HoodCommands.setHoodSetpoint(
            Degrees.of(
                Constants.LauncherC.SHOOTER_HOOD_MAP_SHOOTING.get(
                    ShootTarget.getDistanceToTarget().in(Meters)))),
        ShooterCommands.targetLaunchSpeed(
            RotationsPerSecond.of(
                Constants.LauncherC.FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.get(
                    ShootTarget.getDistanceToTarget().in(Meters)))),
        feedWhenReadyHub());
  }

  private static Command feedWhenReadyPass() {
    return Commands.repeatingSequence(
      Commands.deadline(
        Commands.waitUntil(() -> RobotState.weLockedPass()),
        readyToFeed()),
      Commands.deadline(
        Commands.waitUntil(() -> !RobotState.weLockedPass()),
        feed())
    );
  }

  private static Command feedWhenReadyHub() {
    return Commands.repeatingSequence(
      Commands.deadline(
        Commands.waitUntil(() -> RobotState.weLockedHub()),
        readyToFeed()),
      Commands.deadline(
        Commands.waitUntil(() -> !RobotState.weLockedHub()),
        feed())
    );
  }

  private static Command feed() {
    return Commands.parallel(
        FeederCommands.runPositive(),
        SpindexerCommands.runPositive(),
        Commands.run(() -> System.out.println("ROBO")));
  }

  private static Command readyToFeed() {
    return Commands.parallel(
        SpindexerCommands.idleCommand(),
        Commands.sequence(FeederCommands.retract(), FeederCommands.idleCommand()));
  }
}
