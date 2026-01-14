package frc.lib.utilities.field;

import edu.wpi.first.wpilibj.Timer;

public class Clock {
  public static double time() {
    return Timer.getFPGATimestamp();
  }
}
