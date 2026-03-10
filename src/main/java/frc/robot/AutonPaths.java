package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.lib.utilities.math.GeomUtil;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AutonPaths {
  private static final List<PointTowardsZone> POINT_TOWARDS_EMPTY = Collections.emptyList();
  private static final List<ConstraintsZone> CONSTRAINTS_ZONES_EMPTY = Collections.emptyList();
  private static final List<EventMarker> EVENT_MARKER_EMPTY = Collections.emptyList();

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
    List<Waypoint> waypoints = new ArrayList<>();
    waypoints.add(
        new Waypoint(
            null,
            startingPose.getTranslation(),
            startingPose.getTranslation().plus(new Translation2d(0.2, 0))));
    waypoints.add(
        new Waypoint(
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 2.25, 1.75),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 2, 2),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 1.75, 2.25)));
    waypoints.add(
        new Waypoint(
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 0.2, 2.75),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS + 0.1, 3),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS + 0.2, 3.5)));
    waypoints.add(
        new Waypoint(
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 0.2, 4.25),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS + 0.1, 4.5),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 0.2, 4.75)));
    waypoints.add(
        new Waypoint(
            new Translation2d(Constants.Field.MID_FIELD_X_METERS + 0.2, 4.25),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS + 0.1, 3),
            new Translation2d(Constants.Field.MID_FIELD_X_METERS - 0.2, 4.25)));

    List<RotationTarget> rotationTargets = new ArrayList<>();
    rotationTargets.add(new RotationTarget(0, Rotation2d.fromDegrees(90)));
    rotationTargets.add(new RotationTarget(2, Rotation2d.fromDegrees(90)));
    rotationTargets.add(new RotationTarget(3, Rotation2d.fromDegrees(120)));
    rotationTargets.add(new RotationTarget(5, Rotation2d.fromDegrees(270)));
    rotationTargets.add(new RotationTarget(6, Rotation2d.fromDegrees(270)));

    return new PathPlannerPath(
        waypoints,
        rotationTargets,
        POINT_TOWARDS_EMPTY,
        CONSTRAINTS_ZONES_EMPTY,
        EVENT_MARKER_EMPTY,
        Constants.DriveC.defaultConstraints,
        null,
        new GoalEndState(0.0, Rotation2d.fromDegrees(270)),
        false);
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
