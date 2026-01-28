package frc.lib.utilities.field;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class ShooterRegression {
  public static double[] calculateSpeedAndAngle(Distance distance) {
    double angle = distance.in(Meters) + 1;
    double speed = 0.5 * distance.in(Meters) + 0.2; // Placeholders
    return new double[] {speed, angle};
  }
}
