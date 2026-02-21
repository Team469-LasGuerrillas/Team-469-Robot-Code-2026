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
import frc.lib.subsystems.configs.ServoMotorSubsystemConfig;
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
    public static final double RAMP_DC = 1;
    public static final double UNJAM_DC = -0.5;
    public static final double IDLE_DC = 0.1;

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

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_1_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_2_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_3_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    // private static final ServoMotorSubsystemConfig FOLLOWER_1_SERVO_CONFIG = new
    // ServoMotorSubsystemConfig();
    // private static final ServoMotorSubsystemConfig FOLLOWER_2_SERVO_CONFIG = new
    // ServoMotorSubsystemConfig();
    // private static final ServoMotorSubsystemConfig FOLLOWER_3_SERVO_CONFIG = new
    // ServoMotorSubsystemConfig();

    public static final TalonFXConfiguration LAUNCHER_TALON_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER1_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER2_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER3_CONFIG = new TalonFXConfiguration();

    static {
      LAUNCHER_CONFIG.unitToRotorRatio = 0;

      LAUNCHER_CONFIG.name = "Launcher";
      LAUNCHER_CONFIG.talonCANID = new CANDeviceId(13);

      LAUNCHER_CONFIG.unitToRotorRatio = 1 / 1.25;

      LAUNCHER_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      LAUNCHER_TALON_CONFIG.MotorOutput.ControlTimesyncFreqHz = 0;
      LAUNCHER_TALON_CONFIG.Slot0.kP = 3.3 * Math.pow(10, 38);
      LAUNCHER_TALON_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 40;
      LAUNCHER_TALON_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = 0;
      LAUNCHER_TALON_CONFIG.MotorOutput.PeakForwardDutyCycle = 1;
      LAUNCHER_TALON_CONFIG.MotorOutput.PeakReverseDutyCycle = 0;

      LAUNCHER_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      LAUNCHER_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;

      FOLLOWER_1_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_1_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 70;
      FOLLOWER_2_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_2_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 70;
      FOLLOWER_3_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_3_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 70;

      LAUNCHER_TALON_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      LAUNCHER_CONFIG.fxConfig = LAUNCHER_TALON_CONFIG;

      FOLLOWER_1_CONFIG.config.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      FOLLOWER_2_CONFIG.config.fxConfig.MotorOutput.Inverted =
          InvertedValue.CounterClockwise_Positive;
      FOLLOWER_3_CONFIG.config.fxConfig.MotorOutput.Inverted =
          InvertedValue.CounterClockwise_Positive;

      FOLLOWER_1_CONFIG.inverted = false;
      FOLLOWER_2_CONFIG.inverted = true;
      FOLLOWER_3_CONFIG.inverted = true;

      FOLLOWER_1_CONFIG.config.talonCANID = new CANDeviceId(14);
      FOLLOWER_2_CONFIG.config.talonCANID = new CANDeviceId(15);
      FOLLOWER_3_CONFIG.config.talonCANID = new CANDeviceId(16);

      LAUNCHER_CONFIG.followers =
          new ServoMotorSubsystemWithFollowersConfig.FollowerConfig[] {
            FOLLOWER_1_CONFIG, FOLLOWER_2_CONFIG, FOLLOWER_3_CONFIG
          };
    }

    public static final MotorIO LAUNCHER_MOTOR = new MotorIOTalonFX(LAUNCHER_CONFIG);

    private static final MotorIO FOLLOWER_1_MOTOR = new MotorIOTalonFX(FOLLOWER_1_CONFIG.config);
    private static final MotorIO FOLLOWER_2_MOTOR = new MotorIOTalonFX(FOLLOWER_2_CONFIG.config);
    private static final MotorIO FOLLOWER_3_MOTOR = new MotorIOTalonFX(FOLLOWER_3_CONFIG.config);

    public static final MotorIO[] FOLLOWER_MOTORS =
        new MotorIO[] {FOLLOWER_1_MOTOR, FOLLOWER_2_MOTOR, FOLLOWER_3_MOTOR};
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

    private static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration TURRET_TALON_CONFIG = new TalonFXConfiguration();
    private static final CanCoderConfig TURRET_CANCODER_CONFIG = new CanCoderConfig();

    static {
      TURRET_CANCODER_CONFIG.CANID = new CANDeviceId(6);
      TURRET_CANCODER_CONFIG.config = new CANcoderConfiguration();
      TURRET_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset = -0.36523433383028964;
      TURRET_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
      TURRET_CANCODER_CONFIG.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

      TURRET_TALON_CONFIG.Feedback.FeedbackRemoteSensorID =
          TURRET_CANCODER_CONFIG.CANID.getDeviceNumber();
      TURRET_TALON_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      TURRET_TALON_CONFIG.Feedback.SensorToMechanismRatio = 1;
      TURRET_TALON_CONFIG.Feedback.RotorToSensorRatio = 18.75;

      SERVO_CONFIG.kMaxPositionUnits = TURRERT_MAX.in(Rotations);
      SERVO_CONFIG.kMinPositionUnits = TURRERT_MIN.in(Rotations);

      TURRET_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRERT_MAX.in(Rotations);
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRERT_MIN.in(Rotations);
      TURRET_TALON_CONFIG.Slot0.kP = 0;
      TURRET_TALON_CONFIG.Slot0.kI = 0;
      TURRET_TALON_CONFIG.Slot0.kD = 0;
      TURRET_TALON_CONFIG.Slot0.kS = 0;
      TURRET_TALON_CONFIG.Slot0.kV = 0;
      SERVO_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      TURRET_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      TURRET_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;

      SERVO_CONFIG.talonCANID = new CANDeviceId(20);
      SERVO_CONFIG.canCoderConfig = TURRET_CANCODER_CONFIG;
      SERVO_CONFIG.isFusedCancoder = true;
      SERVO_CONFIG.fxConfig = TURRET_TALON_CONFIG;
    }

    public static final CanCoderIOCanCoder coder = new CanCoderIOCanCoder(TURRET_CANCODER_CONFIG);
    public static final MotorIO motah = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class ClimbC {
    public static final double L1_POS = 67;
    // public static final double L2_POS = probably not gonna be used
    public static final double L3_POS = 6767;
  }

  public static class IntakeC {
    public static final double INTAKE_DC = 1;
    public static final double UNJAM_DC = -0.3;
    public static final double IDLE_DC = 0;
    public static final Angle PIVOT_RAISED = Radians.of(0.33);
    public static final Angle PIVOT_LOWERED = Radians.of(2.187);

    private static final ServoMotorSubsystemWithCancoderConfig DROP_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration INTAKE_PIVOT_TALON_CONFIG =
        new TalonFXConfiguration();
    private static final CanCoderConfig INTAKE_PIVOT_CANCODER_CONFIG = new CanCoderConfig();

    private static final ServoMotorSubsystemWithCancoderConfig ROLLER_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration INTAKE_ROLLER_TALON_CONFIG =
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

      INTAKE_PIVOT_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      INTAKE_PIVOT_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;

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
    public static final double FEEDING_DC = 0.67;
    public static final double REVERSE_DC = -0.67;
    public static final double IDLE_DC = 0.1;

    private static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration SPINDEXER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
      SERVO_CONFIG.talonCANID = new CANDeviceId(11);
      SERVO_CONFIG.fxConfig = SPINDEXER_MOTOR_CONFIG;

      SPINDEXER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      SPINDEXER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      SPINDEXER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;
    }

    public static final MotorIO SPINDEXER_MOTOR = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class FeederC {
    public static final double FEEDING_DC = 1;
    public static final double REVERSE_DC = -1;
    public static final double IDLE_DC = 0;

    private static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
      SERVO_CONFIG.talonCANID = new CANDeviceId(12);
      SERVO_CONFIG.fxConfig = FEEDER_MOTOR_CONFIG;

      FEEDER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      FEEDER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;
    }

    public static final MotorIO FEEDER_MOTOR = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class HoodC {
    public static final Angle HOOD_STOW = Radians.of(0);
    public static final Angle HOOD_MAX = Radians.of(0.5);

    private static final ServoMotorSubsystemConfig HOOD_CONFIG = new ServoMotorSubsystemConfig();
    private static final TalonFXConfiguration HOOD_TALON_CONFIG = new TalonFXConfiguration();

    static {
      HOOD_TALON_CONFIG.Feedback.SensorToMechanismRatio = 111.4;
      HOOD_TALON_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      HOOD_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      HOOD_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      HOOD_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = HOOD_MAX.in(Rotations);
      HOOD_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.05;

      HOOD_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      HOOD_TALON_CONFIG.Slot0.kP = 2500;
      HOOD_TALON_CONFIG.Slot0.kI = 0;
      HOOD_TALON_CONFIG.Slot0.kD = 70;
      HOOD_TALON_CONFIG.Slot0.kS = 11;
      HOOD_TALON_CONFIG.Slot0.kV = 0;
      HOOD_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      HOOD_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
      HOOD_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 20;

      HOOD_CONFIG.talonCANID = new CANDeviceId(17);
      HOOD_CONFIG.fxConfig = HOOD_TALON_CONFIG;
    }

    public static final MotorIO PIVOT_MOTOR = new MotorIOTalonFX(HOOD_CONFIG);
  }
}
