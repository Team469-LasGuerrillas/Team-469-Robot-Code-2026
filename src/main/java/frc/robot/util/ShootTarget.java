package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.utilities.math.GeomUtil;
import frc.lib.utilities.math.ShootAndMove;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class ShootTarget {

  public static Translation2d goal = Constants.Field.RED_HUB;

  public static void updateGoal(Translation2d target, boolean passing) {
    goal =
        ShootAndMove.getTransformed(
            Drive.getInstance().getFieldSpeedsFiltered(),
            Drive.getInstance().getPose(),
            target,
            Constants.TurretC.TURD_CENTER,
            passing);
  }

  public static Translation2d getTranslationToTarget() {
    return new Translation2d(goal.getX(), goal.getY());
  }

  public static Distance getDistanceToTarget() {
    return Meters.of(
        getTranslationToTarget()
            .getDistance(GeomUtil.toTranslation2d(Drive.getInstance().getPose())));
  }
}
