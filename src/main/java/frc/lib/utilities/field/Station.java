package frc.lib.utilities.field;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Station {
  public static boolean isRed() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }
}
