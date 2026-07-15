package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import java.util.function.Supplier;

public class ShooterCommands {
  private static Shooter shooter = Shooter.getInstance();

  public static Command targetLaunchSpeed(Supplier<AngularVelocity> speed) {
    return Commands.sequence(
        Commands.deadline(
            Commands.waitUntil(
                () ->
                    speed.get().in(RotationsPerSecond)
                            - Shooter.getInstance().getSpeed().in(RotationsPerSecond)
                        < Constants.LauncherC.RAMP_SPEED_TOLERANCE.in(RotationsPerSecond)),
            rampSpeed()),
        maintainSpeed(speed));
  }

  public static Command rampSpeed() {
    return Commands.startRun(
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.RAMP_DC),
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.RAMP_DC),
        shooter);
  }

  public static Command maintainSpeed(Supplier<AngularVelocity> speed) {
    return Commands.startRun(
        () -> shooter.setWatermarkTorqueCurrentFOC(speed.get()),
        () -> shooter.setWatermarkTorqueCurrentFOC(speed.get()),
        shooter);
  }

  public static Command idleCommand() {
    return Commands.startRun(
        () -> shooter.setWatermarkTorqueCurrentFOC(RotationsPerSecond.of(0)),
        () -> shooter.setWatermarkTorqueCurrentFOC(RotationsPerSecond.of(0)),
        shooter);
  }

  // public static Command ignoreHubStateCommand() {
  //   return Commands.startRun(
  //       () -> RobotState.setOverrideHubState(true), () -> RobotState.setOverrideHubState(true));
  // }

  // public static Command respectHubStateCommand() {
  //   return Commands.startRun(
  //       () -> RobotState.setOverrideHubState(false), () ->
  // RobotState.setOverrideHubState(false));
  // }
}
