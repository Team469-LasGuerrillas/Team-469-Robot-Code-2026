package frc.robot;

public class RobotState {

  public enum TurretState {
    LOCKED,
    UNLOCKED
  }

  private static TurretState turretState = TurretState.UNLOCKED;

  public static void setTurretState(TurretState state) {
    turretState = state;
  }

  public static TurretState getTurretState() {
    return turretState;
  }

  public enum HoodState {
    LOCKED,
    UNLOCKED
  }

  private static HoodState hoodState = HoodState.UNLOCKED;

  public static void setHoodState(HoodState state) {
    hoodState = state;
  }

  public static HoodState getHoodState() {
    return hoodState;
  }
}
