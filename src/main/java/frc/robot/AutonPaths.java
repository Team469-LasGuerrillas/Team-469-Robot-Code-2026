package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.List;

public class AutonPaths {
  public static PathPlannerPath sixSevenExamplePath(Pose2d startingPose) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            startingPose,
            new Pose2d(2.2 / 2, 0.6, Rotation2d.fromDegrees(90)),
            new Pose2d(2.2, 0, Rotation2d.fromDegrees(180)));

    return new PathPlannerPath(
        waypoints,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(180)));
  }

  public static PathPlannerPath rightAuto(Pose2d startingPose) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            startingPose,
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 2, 1, Rotation2d.fromDegrees(90)),
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 0.2,
                1.5,
                Rotation2d.fromDegrees(90)),
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 0.2, 5, Rotation2d.fromDegrees(90)));

    return new PathPlannerPath(
        waypoints,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(90)));
  }
}
