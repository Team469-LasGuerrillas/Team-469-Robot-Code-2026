package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.utilities.math.GeomUtil;
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

  public static PathPlannerPath rightAutoBlue(Pose2d startingPose) {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            startingPose,
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 2, 1, Rotation2d.fromDegrees(90)),
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 0.2, 2, Rotation2d.fromDegrees(90)),
            new Pose2d(
                (Constants.Field.MAX_FIELD_X.in(Meters) / 2) - 0.2, 6, Rotation2d.fromDegrees(90)));

    return new PathPlannerPath(
        waypoints,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(90)));
  }

  public static PathPlannerPath leftAutoRed(Pose2d startingPose) {
    return rightAutoBlue(GeomUtil.mirror(startingPose)).mirrorPath();
  }

  public static PathPlannerPath climbRed() {
    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(
                Constants.Field.MAX_FIELD_X.in(Meters) - 3,
                Constants.Field.MAX_FIELD_Y.in(Meters) - 3,
                Rotation2d.fromRotations(0.5)),
            new Pose2d(
                Constants.Field.MAX_FIELD_X.in(Meters) - 2,
                Constants.Field.MAX_FIELD_Y.in(Meters) - 3,
                Rotation2d.fromRotations(0.5)));

    return new PathPlannerPath(
        waypoints,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(180)));
  }
}
