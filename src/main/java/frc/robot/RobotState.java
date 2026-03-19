package frc.robot;

import frc.robot.subsystems.drive.Drive;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class RobotState {

  public static boolean weLockedPass() {
    return getTurretState() == TurretState.LOCKED
        && getHoodState() == HoodState.LOCKED
        && (getFlywheelState() == FlywheelState.PASSLOCKED
            || getFlywheelState() == FlywheelState.HUBLOCKED)
        && Math.abs(Drive.getInstance().getPose().getY() - Constants.Field.MID_FIELD_Y_METERS)
            > 0.5;
  }

  private static boolean overrideHubState = false;

  public static boolean weLockedHub() {
    return getTurretState() == TurretState.LOCKED
        && getHoodState() == HoodState.LOCKED
        && getFlywheelState() == FlywheelState.HUBLOCKED
        && (getHubState() == HubState.ACTIVE || overrideHubState);
  }

  public static void setOverrideHubState(boolean override) {
    overrideHubState = override;
  }

  public enum HubState {
    ACTIVE,
    INACTIVE,
  }

  public static HubState getHubState() {
    HubState state;
    if (HubShiftUtil.getShiftedShiftInfo().active()) state = HubState.ACTIVE;
    else state = HubState.INACTIVE;

    Logger.recordOutput("State/HubState", state);
    return state;
  }

  public enum TurretState {
    LOCKED,
    UNLOCKED
  }

  private static TurretState turretState = TurretState.UNLOCKED;

  public static void setTurretState(TurretState state) {
    turretState = state;
    Logger.recordOutput("State/TurretState", state);
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
    Logger.recordOutput("State/HoodState", hoodState);
  }

  public static HoodState getHoodState() {
    return hoodState;
  }

  public enum FlywheelState {
    HUBLOCKED,
    PASSLOCKED,
    UNLOCKED
  }

  private static FlywheelState flywheelState = FlywheelState.UNLOCKED;

  public static void setFlywheelState(FlywheelState state) {
    flywheelState = state;
    Logger.recordOutput("State/FlywheelState", state);
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
