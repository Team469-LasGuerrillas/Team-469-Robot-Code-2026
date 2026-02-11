package frc.robot.subsystems;

public class AutoClimb {

  public enum ClimbLevel {
    L1,
    L3
  }

  private static ClimbLevel queuedClimbPosition = ClimbLevel.L1;

  private static void setQueuedClimbPosition(ClimbLevel newClimbQueuedPosition) {
    queuedClimbPosition = newClimbQueuedPosition;
  }

  private static void autoClimbL3() {
    setQueuedClimbPosition(ClimbLevel.L3);
  }

  private static void autoClimbL1() {
    setQueuedClimbPosition(ClimbLevel.L1);
  }

  public ClimbLevel getQueuedClimbLevel() {
    return queuedClimbPosition;
  }
}
