package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import frc.lib.utilities.math.GeomUtil;
import frc.lib.utilities.math.ShootAndMove;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ShootTarget {

  public static Translation2d goal = Constants.Field.FIELD_CENTER;
  private static Translation2d cachedTarget = new Translation2d();
  private static boolean cachedIsPassing = false;

  public static void updateGoal(Supplier<Translation2d> target, boolean passing) {
    cachedTarget = target.get();
    cachedIsPassing = passing;

    goal =
        ShootAndMove.getTransformed(
            Drive.getInstance().getFieldSpeedsFiltered(),
            Drive.getInstance().getFieldAccelerationsFiltered(),
            Drive.getInstance().getPose(),
            target.get(),
            Constants.TurretC.TURD_CENTER,
            passing);

    Logger.recordOutput("TargetPoint", GeomUtil.toPose2d(goal));
  }

  public static Translation2d getTarget() {
    return cachedTarget;
  }

  public static boolean getIsPassing() {
    return cachedIsPassing;
  }

  public static Translation2d[] timestampedGoals(
      Supplier<Translation2d> target, boolean passing, double[] timestamps) {
    Translation2d[] result = new Translation2d[timestamps.length];

    for (int i = 0; i < timestamps.length; i++) {
      Translation2d timstampGoal =
          ShootAndMove.getTransformed(
              Drive.getInstance().getFieldSpeedsFiltered(),
              Drive.getInstance().getFieldAccelerationsFiltered(),
              Drive.getInstance().getPose(timestamps[i]).get(),
              target.get(),
              Constants.TurretC.TURD_CENTER,
              passing);
      result[i] = new Translation2d(timstampGoal.getX(), timstampGoal.getY());
    }

    return result;
  }

  public static void updateNonDynamicGoal(Translation2d newGoal) {
    goal = newGoal;
  }

  public static Translation2d getTranslationToTarget() {
    return new Translation2d(goal.getX(), goal.getY());
  }

  public static Distance getDistanceToTarget() {
    return Meters.of(
        getTranslationToTarget()
            .getDistance(GeomUtil.toTranslation2d(Turret.getInstance().getTurretPoseFieldSpace())));
  }
}
