package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Turret;
import java.util.function.Supplier;

public class TurretCommands {
  private static Turret turret = Turret.getInstance();

  public static Command targetPoint(Supplier<Translation2d> point) {
    return Commands.startRun(
        () -> turret.setTargetPoint(point.get()), () -> turret.setTargetPoint(point.get()), turret);
  }

  public static Command targetAngle(Angle angle) {
    return Commands.startRun(
        () -> turret.setTargetAngle(angle), () -> turret.setTargetAngle(angle), turret);
  }
}
