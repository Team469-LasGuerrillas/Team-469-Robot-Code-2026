package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class AutonPaths {
  public static PathPlannerPath sixSevenExamplePath() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(1, 0.75, Rotation2d.fromDegrees(45)),
            new Pose2d(1.5, 0.75, Rotation2d.fromDegrees(0)),
            new Pose2d(2.2, 1.5, Rotation2d.fromDegrees(90)));

    return new PathPlannerPath(
        waypoints,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(90)));
  }
}
