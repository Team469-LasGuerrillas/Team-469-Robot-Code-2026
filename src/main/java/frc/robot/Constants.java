// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.configs.CanCoderConfig;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithCancoderConfig;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.implementations.CanCoderIOCanCoder;
import frc.lib.subsystems.implementations.MotorIOTalonFX;
import frc.lib.subsystems.implementations.VisionIOLimelight;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.util.FiducialFilters;
import frc.robot.subsystems.vision.util.FiducialFilters.FiducialModifications;
import java.util.ArrayList;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.REAL;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    // SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveC {
    public static final PathConstraints defaultConstraints =
        new PathConstraints(0.67, 3.3, 4 * Math.PI, 7 * Math.PI);
  }

  public static class Field {
    public static final AprilTagFieldLayout WELDED_FIELD =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance MAX_FIELD_X = Meters.of(WELDED_FIELD.getFieldLength());
    public static final Distance MAX_FIELD_Y = Meters.of(WELDED_FIELD.getFieldWidth());

    public static final Matrix<N3, N1> FIELD_SPEEDS_STDS = VecBuilder.fill(0.067, 0.067, 0.08);
  }

  public static class VisionC {
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.55;
    public static final double MIN_SINGLE_TAG_AREA = 0.33;
    public static final Angle MAX_YAW_ERROR_MT1 = Degrees.of(2.41);
    public static final Angle MAX_YAW_ERROR_MT2 = Degrees.of(1.67);
    public static final Distance MAX_FLOATING_NOCLIP = Meters.of(0.2);

    public static final AngularVelocity BAD_TURRET_ANGULAR_VELOCITY = RadiansPerSecond.of(2);

    public static final AngularVelocity REASONABLE_TURRET_ANGULAR_VELOCITY_MT1 =
        RadiansPerSecond.of(0.1);
    public static final double REASONABLE_TURRET_ANGULAR_VELOCITY_MT1_MULT = 2.17;
    public static final AngularVelocity REASONABLE_TURRET_ANGULAR_VELOCITY_MT2 =
        RadiansPerSecond.of(0.05);
    public static final double REASONABLE_TURRET_ANGULAR_VELOCITY_MT2_MULT = 10;

    public static final ArrayList<Function<PoseObservation, Boolean>> TURRET_REJECTIONS =
        new ArrayList<Function<PoseObservation, Boolean>>();
    public static final ArrayList<UnaryOperator<FiducialModifications>> TURRET_MODIFICATIONS =
        new ArrayList<UnaryOperator<FiducialModifications>>();

    static {
      TURRET_REJECTIONS.add(FiducialFilters.FiducialRejections::badTurretAngularVelocity);

      TURRET_MODIFICATIONS.add(
          FiducialFilters.FiducialModifications.o_withDistrustMt2WhileTurretSpinToFast());
    }

    public static final Pose3d TURD_CENTER =
        new Pose3d(
            0.031613,
            0.183773,
            0.215900,
            new Rotation3d(0, 0, Units.degreesToRadians(180 - 20.220574)));

    public static final VisionIOLimelight DEV_LIMELIGHT =
        VisionIOLimelight.getInstance(
            "limelight-dev",
            new Pose3d(
                Units.inchesToMeters(-3.127),
                Units.inchesToMeters(-8.552080),
                Units.inchesToMeters(6.158800 - 0.125),
                new Rotation3d(0, Units.degreesToRadians(26), Units.degreesToRadians(15))));

    public static final VisionIOLimelight TURD_LIMELIGHT =
        VisionIOLimelight.getInstance(
            "limelight-turd",
            new Pose3d(
                0.081363,
                0.244337,
                0.299212 - Units.inchesToMeters(0.125),
                new Rotation3d(
                    0, Units.degreesToRadians(28), Units.degreesToRadians(180 - 20.220574))));
  }

  public static class LauncherC {
    public static final double SHOOT_VOLTAGE = 1;
    public static final double OUTTAKE_VOLTAGE = SHOOT_VOLTAGE * -1;
    public static final double DEFAULT_VOLTAGE = 0;

    public static double phaseDelay;

    public static final InterpolatingDoubleTreeMap FLYWHEEL_SHOT_SPEEDMAP_SHOOTING =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP_SHOOTING =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap SHOOTER_HOOD_MAP_SHOOTING =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap FLYWHEEL_SHOT_SPEEDMAP_PASSING =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap TIME_OF_FLIGHT_MAP_PASSING =
        new InterpolatingDoubleTreeMap();

    public static final InterpolatingDoubleTreeMap SHOOTER_HOOD_MAP_PASSING =
        new InterpolatingDoubleTreeMap();

    static {
      phaseDelay = 67;

      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.7, 6.7);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.72, 6.72);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.73, 6.73);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(53.2, 348.2);

      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.7, 6.7);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.72, 6.72);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.73, 6.73);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(6.71, 6.71);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(53.2, 348.2);

      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(12.1, 21.3);

      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);
      TIME_OF_FLIGHT_MAP_PASSING.put(12.1, 21.3);

      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_SHOOTING.put(43.9, 89.2);

      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
      SHOOTER_HOOD_MAP_PASSING.put(43.9, 89.2);
    }

    public static final ServoMotorSubsystemWithFollowersConfig LAUNCHER_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_2_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_3_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final TalonFXConfiguration LAUNCHER_TALON_CONFIG = new TalonFXConfiguration();

    public static final TalonFXConfiguration LAUNCHER_TALON_FOLLOWER1_CONFIG =
        new TalonFXConfiguration();

    public static final TalonFXConfiguration LAUNCHER_TALON_FOLLOWER2_CONFIG =
        new TalonFXConfiguration();

    public static final TalonFXConfiguration LAUNCHER_TALON_FOLLOWER3_CONFIG =
        new TalonFXConfiguration();

    static {
      LAUNCHER_CONFIG.unitToRotorRatio = 0;

      LAUNCHER_CONFIG.name = "Launcher";
      LAUNCHER_CONFIG.talonCANID = new CANDeviceId(0);

      LAUNCHER_TALON_CONFIG.Slot0.kP = 6.7;
      LAUNCHER_TALON_CONFIG.Slot0.kI = 67;
      LAUNCHER_TALON_CONFIG.Slot0.kD = 6767;

      LAUNCHER_CONFIG.fxConfig = LAUNCHER_TALON_CONFIG;
    }
  }

  public static class TurretC {

    public static final Angle TURRERT_MAX = Rotations.of(0.6);
    public static final Angle TURRERT_MIN = Rotations.of(-0.6);

    public static final Pose3d TURD_CENTER =
        new Pose3d(
            0.031613,
            0.183773,
            0.215900,
            new Rotation3d(0, 0, Units.degreesToRadians(180 - 20.220574)));

    public static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    public static final TalonFXConfiguration EXAMPE_TALON_CONFIG = new TalonFXConfiguration();
    public static final CanCoderConfig EXAMPE_CANCODER_CONFIG = new CanCoderConfig();

    static {
      EXAMPE_CANCODER_CONFIG.CANID = new CANDeviceId(6);
      EXAMPE_CANCODER_CONFIG.config = new CANcoderConfiguration();
      EXAMPE_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset = -0.36523433383028964;
      EXAMPE_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
      EXAMPE_CANCODER_CONFIG.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

      EXAMPE_TALON_CONFIG.Feedback.FeedbackRemoteSensorID =
          EXAMPE_CANCODER_CONFIG.CANID.getDeviceNumber();
      EXAMPE_TALON_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      EXAMPE_TALON_CONFIG.Feedback.SensorToMechanismRatio = 1;
      EXAMPE_TALON_CONFIG.Feedback.RotorToSensorRatio = 18.75;

      SERVO_CONFIG.kMaxPositionUnits = TURRERT_MAX.in(Rotations);
      SERVO_CONFIG.kMinPositionUnits = TURRERT_MIN.in(Rotations);

      EXAMPE_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      EXAMPE_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRERT_MAX.in(Rotations);
      EXAMPE_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRERT_MIN.in(Rotations);
      EXAMPE_TALON_CONFIG.Slot0.kP = 0;
      EXAMPE_TALON_CONFIG.Slot0.kI = 0;
      EXAMPE_TALON_CONFIG.Slot0.kD = 0;
      EXAMPE_TALON_CONFIG.Slot0.kS = 0;
      EXAMPE_TALON_CONFIG.Slot0.kV = 0;
      SERVO_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      SERVO_CONFIG.talonCANID = new CANDeviceId(20);
      SERVO_CONFIG.canCoderConfig = EXAMPE_CANCODER_CONFIG;
      SERVO_CONFIG.isFusedCancoder = true;
      SERVO_CONFIG.fxConfig = EXAMPE_TALON_CONFIG;
    }

    public static final CanCoderIOCanCoder coder = new CanCoderIOCanCoder(EXAMPE_CANCODER_CONFIG);
    public static final MotorIO motah = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class ClimbC {
    public static final double L1_POS = 67;
    // public static final double L2_POS = probably not gonna be used
    public static final double L3_POS = 6767;
  }

  public static class IntakeC {
    public static final double INTAKE_VOLTAGE = 1;
    public static final double OUTTAKE_VOLTAGE = INTAKE_VOLTAGE * -1;
    public static final double DEFAULT_VOLTAGE = 0;
    public static final double PIVOT_DEFAULT = 67;
    public static final Angle PIVOT_RAISED = Radians.of(0.33);
    public static final Angle PIVOT_LOWERED = Radians.of(2.187);
    public static final double IM_PREPARED_FOR_FUTURE_STUFF = 86;

    public static final ServoMotorSubsystemWithCancoderConfig DROP_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    public static final ServoMotorSubsystemWithCancoderConfig ROLLER_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    public static final TalonFXConfiguration INTAKE_PIVOT_TALON_CONFIG = new TalonFXConfiguration();
    public static final CanCoderConfig INTAKE_PIVOT_CANCODER_CONFIG = new CanCoderConfig();

    public static final TalonFXConfiguration INTAKE_ROLLER_TALON_CONFIG =
        new TalonFXConfiguration();

    static {
      INTAKE_PIVOT_CANCODER_CONFIG.CANID = new CANDeviceId(5);
      INTAKE_PIVOT_CANCODER_CONFIG.config = new CANcoderConfiguration();
      INTAKE_PIVOT_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset =
          Units.radiansToRotations(-1.33) + 1;
      INTAKE_PIVOT_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.CounterClockwise_Positive;
      INTAKE_PIVOT_CANCODER_CONFIG.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

      INTAKE_PIVOT_TALON_CONFIG.Feedback.FeedbackRemoteSensorID =
          INTAKE_PIVOT_CANCODER_CONFIG.CANID.getDeviceNumber();
      INTAKE_PIVOT_TALON_CONFIG.Feedback.FeedbackSensorSource =
          FeedbackSensorSourceValue.FusedCANcoder;
      INTAKE_PIVOT_TALON_CONFIG.Feedback.SensorToMechanismRatio = 1;
      INTAKE_PIVOT_TALON_CONFIG.Feedback.RotorToSensorRatio = 32;
      INTAKE_PIVOT_TALON_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 100;
      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -100;

      INTAKE_PIVOT_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kP = 300;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kI = 0;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kD = 50;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kS = 0;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kV = 0;
      DROP_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      DROP_CONFIG.talonCANID = new CANDeviceId(9);
      DROP_CONFIG.canCoderConfig = INTAKE_PIVOT_CANCODER_CONFIG;
      DROP_CONFIG.isFusedCancoder = true;
      DROP_CONFIG.fxConfig = INTAKE_PIVOT_TALON_CONFIG;

      ROLLER_CONFIG.talonCANID = new CANDeviceId(10);
      ROLLER_CONFIG.fxConfig = INTAKE_ROLLER_TALON_CONFIG;
    }

    public static final CanCoderIOCanCoder coder =
        new CanCoderIOCanCoder(INTAKE_PIVOT_CANCODER_CONFIG);
    public static final MotorIO PIVOT_MOTOR = new MotorIOTalonFX(DROP_CONFIG);

    public static final MotorIO ROLLER_MOTOR = new MotorIOTalonFX(ROLLER_CONFIG);
  }

  public static class SpindexerC {
    public static final double CLOCKWISE_VOLTAGE = 1;
    public static final double REVERSE_VOLTAGE = CLOCKWISE_VOLTAGE * -1;
    public static final double DEFAULT_VOLTAGE = 0;

    public static final TalonFXConfiguration SPINDEXER_MOTOR_CONFIG = new TalonFXConfiguration();
    public static final CanCoderConfig SPINDEXER_CANCODER_CONFIG = new CanCoderConfig();
  }
}
