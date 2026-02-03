package frc.robot;

public class RobotState {

  public enum TurretState {
    ON_TARGET,
    OFF_TARGET
  }

  private static TurretState turretState = TurretState.OFF_TARGET;

  public static void setTurretState(TurretState state) {}
}
