package frc.robot.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {
  private static Shooter shooter = Shooter.getInstance();

  public static Command rampSpeed() {
    return Commands.startRun(
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.RAMP_DC),
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.RAMP_DC),
        shooter);
  }

  public static Command maintainSpeed() {
    return Commands.startRun(
        () -> shooter.setWatermarkTorqueCurrentFOC(RadiansPerSecond.of(350)),
        () -> shooter.setWatermarkTorqueCurrentFOC(RadiansPerSecond.of(350)),
        shooter);
  }

  public static Command idleCommand() {
    return Commands.startRun(
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.IDLE_DC),
        () -> shooter.setOpenLoopDutyCycle(Constants.LauncherC.IDLE_DC),
        shooter);
  }
}
