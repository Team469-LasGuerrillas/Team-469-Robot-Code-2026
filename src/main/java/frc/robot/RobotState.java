package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

public class RobotState {

  public static boolean weLockedPass() {
    return getTurretState() == TurretState.LOCKED
        && getHoodState() == HoodState.LOCKED
        && (getFlywheelState() == FlywheelState.PASSLOCKED
            || getFlywheelState() == FlywheelState.HUBLOCKED);
  }

  public static boolean weLockedHub() {
    return getTurretState() == TurretState.LOCKED
        && getHoodState() == HoodState.LOCKED
        && getFlywheelState() == FlywheelState.HUBLOCKED;
  }

  public enum TurretState {
    LOCKED,
    UNLOCKED
  }

  @AutoLogOutput(key = "State/TurretState")
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

  @AutoLogOutput(key = "State/HoodState")
  private static HoodState hoodState = HoodState.UNLOCKED;

  public static void setHoodState(HoodState state) {
    hoodState = state;
  }

  public static HoodState getHoodState() {
    return hoodState;
  }

  public enum FlywheelState {
    HUBLOCKED,
    PASSLOCKED,
    UNLOCKED
  }

  @AutoLogOutput(key = "State/FlywheelState")
  private static FlywheelState flywheelState = FlywheelState.UNLOCKED;

  public static void setFlywheelState(FlywheelState state) {
    flywheelState = state;
  }

  public static FlywheelState getFlywheelState() {
    return flywheelState;
  }

  public enum ClimbElevatorState {
    L1,
    L3,
    STOWED,
    OFFTARGET
  }

  @AutoLogOutput(key = "State/ClimbElevatorState")
  private static ClimbElevatorState climbelevatorState = ClimbElevatorState.OFFTARGET;

  public static void setClimbElevatorState(ClimbElevatorState state) {
    climbelevatorState = state;
  }

  public static ClimbElevatorState getClimbElevatorState() {
    return climbelevatorState;
  }

  public enum ClimbHookState {
    STOWED,
    DEPLOYED
  }

  @AutoLogOutput(key = "State/ClimbHookState")
  private static ClimbHookState climbHookState = ClimbHookState.STOWED;

  public static void setClimbHookDeployed() {
    climbHookState = ClimbHookState.DEPLOYED;
  }

  public static ClimbHookState getClimbHookState() {
    return climbHookState;
  }

  public enum IntakeState {
    STOWED,
    DEPLOYED,
    OFFTARGET
  }

  @AutoLogOutput(key = "State/IntakeState")
  private static IntakeState intakeState = IntakeState.STOWED;

  public static void setIntakeState(IntakeState state) {
    intakeState = state;
  }

  public static IntakeState getIntakeState() {
    return intakeState;
  }

  public enum DriveState {
    TELEOP,
    PATHPLANNER,
    TOPOINT_ALIGNING,
    TOPOINT_ONTARGET
  }

  @AutoLogOutput(key = "State/DriveState")
  private static DriveState driveState = DriveState.TELEOP;

  public static void setDrivestate(DriveState state) {
    driveState = state;
  }

  public static DriveState getDriveState() {
    return driveState;
  }
}
