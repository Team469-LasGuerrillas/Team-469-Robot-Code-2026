// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

  public static final SwerveModulePosition[] EMPTY_MODULE_POSITIONS = new SwerveModulePosition[4];

  static {
    for (int i = 0; i < 4; i++) {
      EMPTY_MODULE_POSITIONS[i] = new SwerveModulePosition();
    }
  }

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
        new PathConstraints(1, 2, 3 * Math.PI, 10 * Math.PI);

    public static final PPHolonomicDriveController PP_CONTROLLER =
        new PPHolonomicDriveController(
            new PIDConstants(8, 0.0, 0.3), new PIDConstants(5.3, 0.0, 0.6));
  }

  public static class Field {
    public static final AprilTagFieldLayout WELDED_FIELD =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance MAX_FIELD_X = Meters.of(WELDED_FIELD.getFieldLength());
    public static final Distance MAX_FIELD_Y = Meters.of(WELDED_FIELD.getFieldWidth());
    public static final Distance MID_FIELD_X = MAX_FIELD_X.div(2); // also red max
    public static final Distance MID_FIELD_Y = MAX_FIELD_Y.div(2);
    public static final double MID_FIELD_X_METERS = MID_FIELD_X.in(Meters);
    public static final double MID_FIELD_Y_METERS = MID_FIELD_Y.in(Meters);

    public static final Matrix<N3, N1> FIELD_SPEEDS_STDS = VecBuilder.fill(0.067, 0.067, 0.08);
    public static final Matrix<N3, N1> FIELD_ACCELERATIONS_STDS = VecBuilder.fill(0.24, 0.24, 0.24);
    public static final Matrix<N3, N1> TURRET_SPEEDS_STDS = VecBuilder.fill(0.1, 0.1, 0.1);
    public static final Matrix<N3, N1> TURRET_MOTOR_SPEEDS_STDS =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(80));
    public static final Matrix<N3, N1> TURRET_TARGET_SPEEDS_STDS =
        VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(67));

    public static final Distance BLUE_TRENCH_SCORING = Inches.of(182.11);
    public static final Distance RED_TRENCH_SCORING = MAX_FIELD_X.minus(Inches.of(182.11));

    public static final Distance REGULAR_DECAPITATION_ZONE = Meters.of(0.5);

    public static final double DECAPITATION_SPEED_FACTOR = 0.2;

    public static final Translation2d BLUE_HUB =
        new Translation2d(BLUE_TRENCH_SCORING.in(Meters), MID_FIELD_Y.in(Meters));
    public static final Translation2d RED_HUB =
        new Translation2d(RED_TRENCH_SCORING.in(Meters), MID_FIELD_Y.in(Meters));

    public static final Distance PASS_WALL_DISTANCE = Meters.of(3.5);
  }

  public static class VisionC {
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.55;
    public static final double MIN_SINGLE_TAG_AREA = 0.33;
    public static final Angle MAX_YAW_ERROR_MT1 = Degrees.of(2.41);
    public static final Angle MAX_YAW_ERROR_MT2 = Degrees.of(2);
    public static final Distance MAX_FLOATING_NOCLIP = Meters.of(1.0);

    public static final AngularVelocity BAD_TURRET_ANGULAR_VELOCITY = DegreesPerSecond.of(90);

    public static final AngularVelocity REASONABLE_TURRET_ANGULAR_VELOCITY_MT1 =
        DegreesPerSecond.of(20);
    public static final double REASONABLE_TURRET_ANGULAR_VELOCITY_MT1_MULT = 12;
    public static final AngularVelocity REASONABLE_TURRET_ANGULAR_VELOCITY_MT2 =
        DegreesPerSecond.of(20);
    public static final double REASONABLE_TURRET_ANGULAR_VELOCITY_MT2_MULT = 10;

    public static final AngularVelocity REASONABLE_DRIVE_ANGULAR_VELOCITY_MT2 =
        DegreesPerSecond.of(200);
    public static final double REASONABLE_DRIVE_ANGULAR_VELOCITY_MT2_MULT = 20;

    public static final ArrayList<Function<PoseObservation, Boolean>> TURRET_REJECTIONS =
        new ArrayList<Function<PoseObservation, Boolean>>();
    public static final ArrayList<UnaryOperator<FiducialModifications>> TURRET_MODIFICATIONS =
        new ArrayList<UnaryOperator<FiducialModifications>>();

    public static final ArrayList<UnaryOperator<FiducialModifications>> LL3G_MODIFICATIONS =
        new ArrayList<UnaryOperator<FiducialModifications>>();

    static {
      TURRET_REJECTIONS.add(FiducialFilters.FiducialRejections::badTurretAngularVelocity);

      TURRET_MODIFICATIONS.add(
          FiducialFilters.FiducialModifications.o_withDistrustMt2WhileTurretSpinToFast());
      TURRET_MODIFICATIONS.add(FiducialFilters.FiducialModifications.o_withDistrustYaw());
      TURRET_MODIFICATIONS.add(FiducialFilters.FiducialModifications.o_withMultiplyAllResults());
      // TURRET_MODIFICATIONS.add(FiducialFilters.FiducialModifications.o_withDistrustMt1());

      LL3G_MODIFICATIONS.add(
          FiducialFilters.FiducialModifications.o_withDistrustMt2WhileDriveSpinToFast());
    }

    public static final Pose3d TURD_CENTER =
        new Pose3d(0.031613, 0.183773, 0.215900, new Rotation3d(0, 0, Units.degreesToRadians(0)));

    public static final VisionIOLimelight LIMELIGHT_RIGHT =
        VisionIOLimelight.getInstance(
            "limelight-right",
            new Pose3d(
                -0.139700,
                -0.352969,
                0.313057 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-90))));

    public static final VisionIOLimelight LIMELIGHT_B =
        VisionIOLimelight.getInstance(
            "limelight-turd",
            new Pose3d(
                -0.250329,
                0.144134,
                0.427293 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(18), Units.degreesToRadians(180 + 25))));

    public static final VisionIOLimelight LIMELIGHT_C =
        VisionIOLimelight.getInstance(
            "limelight-c",
            new Pose3d(
                -0.268509,
                0.134876,
                0.274292 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(20), Units.degreesToRadians(180 - 25))));

    public static final VisionIOLimelight LIMELIGHT_LEFT =
        VisionIOLimelight.getInstance(
            "limelight-left",
            new Pose3d(
                -0.228600,
                0.359812,
                0.432361 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(90))));

    public static final VisionIOLimelight LIMELIGHT_CLIMB =
        VisionIOLimelight.getInstance(
            "limelight-climb",
            new Pose3d(
                -0.215665,
                0.289118 - Units.inchesToMeters(0.25),
                0.522026 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-15))));

    public static final VisionIOLimelight TURD_LIMELIGHT =
        VisionIOLimelight.getInstance(
            "limelight-turdeeeee",
            new Pose3d(
                0.081752,
                -0.201245 - 0.040055 - 0.040055 + 0.165100,
                0.501594 - Units.inchesToMeters(0.125),
                new Rotation3d(0, Units.degreesToRadians(13.396331), Units.rotationsToRadians(0))));
  }

  public static class LauncherC {
    public static final double RAMP_DC = 0.95;
    public static final double UNJAM_DC = -0.5;
    public static final double IDLE_DC = 0.0;

    public static final AngularVelocity HUB_SPEED_TOLERANCE = RotationsPerSecond.of(6.7);
    public static final AngularVelocity PASS_SPEED_TOLERANCE = RotationsPerSecond.of(44);
    public static final AngularVelocity RAMP_SPEED_TOLERANCE = RotationsPerSecond.of(15);

    public static final double FEEDER_LEAD_TIME_SECONDS = 0.07;

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

      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(0.1, 35.0);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(3.5, 48.0);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(4.5, 52.7);
      FLYWHEEL_SHOT_SPEEDMAP_SHOOTING.put(6.3, 62.0);

      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(0.5, 32.0);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(2.0, 38.0);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(5.0, 60.0);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(7.0, 80.0);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(10.0, 85.0);
      FLYWHEEL_SHOT_SPEEDMAP_PASSING.put(20.0, 90.0);

      TIME_OF_FLIGHT_MAP_SHOOTING.put(0.2, 0.3);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(1.0, 0.74);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(1.8, 1.0);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(2.5, 1.14);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(3.5, 1.27);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(5.0, 1.35);
      TIME_OF_FLIGHT_MAP_SHOOTING.put(6.0, 1.45);

      TIME_OF_FLIGHT_MAP_PASSING.put(1.0, 0.8);
      TIME_OF_FLIGHT_MAP_PASSING.put(4.0, 1.1);
      TIME_OF_FLIGHT_MAP_PASSING.put(7.0, 1.4);
      TIME_OF_FLIGHT_MAP_PASSING.put(20.0, 1.8);

      SHOOTER_HOOD_MAP_SHOOTING.put(0.2, 0.0);
      SHOOTER_HOOD_MAP_SHOOTING.put(0.8, 0.1);
      SHOOTER_HOOD_MAP_SHOOTING.put(1.7, 0.5);
      SHOOTER_HOOD_MAP_SHOOTING.put(2.2, 3.0);
      SHOOTER_HOOD_MAP_SHOOTING.put(3.75, 8.0);
      SHOOTER_HOOD_MAP_SHOOTING.put(5.0, 13.0);

      SHOOTER_HOOD_MAP_PASSING.put(0.5, 0.0);
      SHOOTER_HOOD_MAP_PASSING.put(2.0, 10.0);
      SHOOTER_HOOD_MAP_PASSING.put(4.0, 18.0);
      SHOOTER_HOOD_MAP_PASSING.put(7.0, 22.0);
      SHOOTER_HOOD_MAP_PASSING.put(20.0, 22.0);
    }

    public static final ServoMotorSubsystemWithFollowersConfig LAUNCHER_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_1_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_2_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final ServoMotorSubsystemWithFollowersConfig.FollowerConfig FOLLOWER_3_CONFIG =
        new ServoMotorSubsystemWithFollowersConfig.FollowerConfig();

    public static final TalonFXConfiguration LAUNCHER_TALON_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER1_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER2_CONFIG = new TalonFXConfiguration();
    public static final TalonFXConfiguration TALON_FOLLOWER3_CONFIG = new TalonFXConfiguration();

    static {
      LAUNCHER_CONFIG.name = "Launcher";
      LAUNCHER_CONFIG.talonCANID = new CANDeviceId(14, "469CanivoreB");

      LAUNCHER_CONFIG.unitToRotorRatio = 1 / 1.25;

      LAUNCHER_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      LAUNCHER_TALON_CONFIG.MotorOutput.ControlTimesyncFreqHz = 0;
      LAUNCHER_TALON_CONFIG.Slot0.kP = 4.69;
      LAUNCHER_TALON_CONFIG.Slot0.kI = 0;
      LAUNCHER_TALON_CONFIG.Slot0.kD = 0;
      LAUNCHER_TALON_CONFIG.Slot0.kV = 0.267;

      LAUNCHER_TALON_CONFIG.Slot1.kP = 1;
      LAUNCHER_TALON_CONFIG.Slot1.kI = 0;
      LAUNCHER_TALON_CONFIG.Slot1.kD = 0;
      // LAUNCHER_TALON_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = 120;
      LAUNCHER_TALON_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -10;
      LAUNCHER_TALON_CONFIG.MotorOutput.PeakForwardDutyCycle = 1;
      LAUNCHER_TALON_CONFIG.MotorOutput.PeakReverseDutyCycle = 0;

      LAUNCHER_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      LAUNCHER_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 40;

      LAUNCHER_TALON_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      FOLLOWER_1_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_1_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 40;
      FOLLOWER_2_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_2_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 40;
      FOLLOWER_3_CONFIG.config.fxConfig.CurrentLimits.StatorCurrentLimit = 120;
      FOLLOWER_3_CONFIG.config.fxConfig.CurrentLimits.SupplyCurrentLimit = 40;

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

      FOLLOWER_1_CONFIG.config.talonCANID = new CANDeviceId(13, "469CanivoreB");
      FOLLOWER_2_CONFIG.config.talonCANID = new CANDeviceId(15, "469CanivoreB");
      FOLLOWER_3_CONFIG.config.talonCANID = new CANDeviceId(16, "469CanivoreB");

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

    public static final Angle TURRET_TOLERANCE = Degrees.of(6.7);

    public static final Angle TURRERT_MAX = Rotations.of(0.35);
    public static final Angle TURRERT_MIN = Rotations.of(-0.9);

    public static final double WRAPAROUND_PREDICTION_FACOTR = 0.2;

    public static final Pose3d TURD_CENTER =
        new Pose3d(
            -0.107950,
            -0.158750,
            0.414338,
            new Rotation3d(0, 0, Units.rotationsToRadians(0.25 - 0.0262)));

    public static final Pose3d TURD_CENTER_WITHOUT_ROTATION =
        new Pose3d(-0.107950, -0.158750, 0.414338, new Rotation3d());

    private static final ServoMotorSubsystemConfig SERVO_CONFIG = new ServoMotorSubsystemConfig();
    private static final TalonFXConfiguration TURRET_TALON_CONFIG = new TalonFXConfiguration();
    private static final CanCoderConfig TURRETA_CANCODER_CONFIG = new CanCoderConfig();
    private static final CanCoderConfig TURRETB_CANCODER_CONFIG = new CanCoderConfig();

    public static final double ROTATION_SPEED_FF = 0; // 16 / 100.0; // Amps per degree per second
    public static final double LEAD_SHOT_OFFSET = 0; // Rotations per RPS of turret base

    static {
      TURRETA_CANCODER_CONFIG.CANID = new CANDeviceId(6, "469CanivoreB");
      TURRETA_CANCODER_CONFIG.config = new CANcoderConfiguration();
      TURRETA_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset = 0;
      TURRETA_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
      TURRETA_CANCODER_CONFIG.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

      TURRETB_CANCODER_CONFIG.CANID = new CANDeviceId(7, "469CanivoreB");
      TURRETB_CANCODER_CONFIG.config = new CANcoderConfiguration();
      TURRETB_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset = 0;
      TURRETB_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
      TURRETB_CANCODER_CONFIG.config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

      TURRET_TALON_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      TURRET_TALON_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      // 168t main gear, 26t cancoder A,
      TURRET_TALON_CONFIG.Feedback.SensorToMechanismRatio = ((52.0 / 10.0) * (168.0 / 18.0));
      TURRET_TALON_CONFIG.Feedback.RotorToSensorRatio = 1; // ((52.0 / 10.0) * (168.0 / 18.0));

      SERVO_CONFIG.kMaxPositionUnits = TURRERT_MAX.in(Rotations);
      SERVO_CONFIG.kMinPositionUnits = TURRERT_MIN.in(Rotations);

      TURRET_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = TURRERT_MAX.in(Rotations);
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = TURRERT_MIN.in(Rotations);
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      TURRET_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      TURRET_TALON_CONFIG.Slot0.kP = 3310; // 3310
      TURRET_TALON_CONFIG.Slot0.kI = 0;
      TURRET_TALON_CONFIG.Slot0.kD = 98.7; // 98.7
      TURRET_TALON_CONFIG.Slot0.kS = 0.5;
      TURRET_TALON_CONFIG.Slot0.kV = (16.0 / 100.0) * 360.0 * 2;
      TURRET_TALON_CONFIG.Slot0.kA = 0;
      SERVO_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      TURRET_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      TURRET_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 70;

      SERVO_CONFIG.talonCANID = new CANDeviceId(20, "469CanivoreB");
      // SERVO_CONFIG.canCoderConfig = TURRETA_CANCODER_CONFIG;
      // SERVO_CONFIG.isFusedCancoder = true;
      SERVO_CONFIG.fxConfig = TURRET_TALON_CONFIG;
    }

    public static final CanCoderIOCanCoder coderA = new CanCoderIOCanCoder(TURRETA_CANCODER_CONFIG);
    public static final CanCoderIOCanCoder coderB = new CanCoderIOCanCoder(TURRETB_CANCODER_CONFIG);
    public static final MotorIO motah = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class ClimbC {
    public static final Angle CLIMB_STOW = Rotations.of(-0.01);
    public static final Angle CLIMB_CLEAR = Rotations.of(5);
    public static final Angle CLIMB_MAX = Rotations.of(9);

    private static final ServoMotorSubsystemConfig CLIMB_CONFIG = new ServoMotorSubsystemConfig();
    private static final TalonFXConfiguration CLIMB_TALON_CONFIG = new TalonFXConfiguration();

    static {
      CLIMB_TALON_CONFIG.Feedback.SensorToMechanismRatio = 22.5;
      CLIMB_TALON_CONFIG.Feedback.RotorToSensorRatio = 1;
      CLIMB_TALON_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      // CLIMB_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // CLIMB_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // CLIMB_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold = CLIMB_MAX.in(Rotations);
      // CLIMB_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      // CLIMB_STOW.in(Rotations);

      CLIMB_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      CLIMB_TALON_CONFIG.Slot0.kP = 0; // 10000 Jackhammer
      CLIMB_TALON_CONFIG.Slot0.kI = 0;
      CLIMB_TALON_CONFIG.Slot0.kD = 0;
      CLIMB_TALON_CONFIG.Slot0.kS = 0;
      CLIMB_TALON_CONFIG.Slot0.kV = 0;
      CLIMB_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      CLIMB_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 0.469;
      CLIMB_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 0.469;

      CLIMB_CONFIG.talonCANID = new CANDeviceId(18);
      CLIMB_CONFIG.fxConfig = CLIMB_TALON_CONFIG;
    }

    // public static final MotorIO CLIMB_MOTOR = new MotorIOTalonFX(CLIMB_CONFIG);
  }

  public static class IntakeC {
    public static final double INTAKE_DC = 1;
    public static final double UNJAM_DC = -0.3;
    public static final double IDLE_DC = 0;
    // public static final Angle PIVOT_RAISED = Radians.of(0.33);
    public static final Angle PIVOT_LOWERED = Radians.of(2.0);
    public static final Angle PIVOT_RAISED = Degrees.of(45);
    public static final Angle PIVOT_AGITATE = Degrees.of(45);

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
      INTAKE_PIVOT_CANCODER_CONFIG.config.MagnetSensor.MagnetOffset = 1.229004;
      INTAKE_PIVOT_CANCODER_CONFIG.config.MagnetSensor.SensorDirection =
          SensorDirectionValue.Clockwise_Positive;
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
      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          PIVOT_LOWERED.in(Rotations);
      INTAKE_PIVOT_TALON_CONFIG.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

      INTAKE_PIVOT_TALON_CONFIG.ClosedLoopGeneral.ContinuousWrap = false;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kP = 440;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kI = 0;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kD = 50;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kS = 0;
      INTAKE_PIVOT_TALON_CONFIG.Slot0.kV = 0;
      DROP_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      INTAKE_PIVOT_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      INTAKE_PIVOT_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;

      DROP_CONFIG.talonCANID = new CANDeviceId(9);
      DROP_CONFIG.canCoderConfig = INTAKE_PIVOT_CANCODER_CONFIG;
      DROP_CONFIG.isFusedCancoder = true;
      DROP_CONFIG.fxConfig = INTAKE_PIVOT_TALON_CONFIG;

      INTAKE_ROLLER_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      INTAKE_ROLLER_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 45;

      ROLLER_CONFIG.talonCANID = new CANDeviceId(10);
      ROLLER_CONFIG.fxConfig = INTAKE_ROLLER_TALON_CONFIG;
    }

    public static final CanCoderIOCanCoder coder =
        new CanCoderIOCanCoder(INTAKE_PIVOT_CANCODER_CONFIG);
    public static final MotorIO PIVOT_MOTOR = new MotorIOTalonFX(DROP_CONFIG);

    public static final MotorIO ROLLER_MOTOR = new MotorIOTalonFX(ROLLER_CONFIG);
  }

  public static class SpindexerC {
    public static final double FEEDING_DC = 1.0;
    public static final double REVERSE_DC = -1.0;
    public static final double IDLE_DC = 0.0;
    public static final double IDLE_REVERSE_DC = -0.0;

    private static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration SPINDEXER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
      SERVO_CONFIG.talonCANID = new CANDeviceId(11);
      SERVO_CONFIG.fxConfig = SPINDEXER_MOTOR_CONFIG;

      SPINDEXER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      SPINDEXER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 100;
      SPINDEXER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 60;
    }

    public static final MotorIO SPINDEXER_MOTOR = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class FeederC {
    public static final double FEEDING_DC = 1;
    public static final double REVERSE_DC = -1;
    public static final double IDLE_REVERSE_DC = -0.0;
    public static final double IDLE_DC = 0;

    private static final ServoMotorSubsystemWithCancoderConfig SERVO_CONFIG =
        new ServoMotorSubsystemWithCancoderConfig();
    private static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = new TalonFXConfiguration();

    static {
      SERVO_CONFIG.talonCANID = new CANDeviceId(12, "469CanivoreB");
      SERVO_CONFIG.fxConfig = FEEDER_MOTOR_CONFIG;

      FEEDER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      FEEDER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      FEEDER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 120;
      FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLimit = 50;

      // FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerLimit = 60;
      // FEEDER_MOTOR_CONFIG.CurrentLimits.SupplyCurrentLowerTime = 1;
    }

    public static final MotorIO FEEDER_MOTOR = new MotorIOTalonFX(SERVO_CONFIG);
  }

  public static class HoodC {
    public static final Angle HOOD_TOLERANCE = Degrees.of(1.67);

    public static final Angle HOOD_STOW = Radians.of(0);
    public static final Angle HOOD_MAX = Radians.of(0.436);

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
      HOOD_TALON_CONFIG.Slot0.kP = 3700;
      HOOD_TALON_CONFIG.Slot0.kI = 0;
      HOOD_TALON_CONFIG.Slot0.kD = 80;
      HOOD_TALON_CONFIG.Slot0.kS = 3;
      HOOD_TALON_CONFIG.Slot0.kV = 0;
      HOOD_CONFIG.outputMode = ClosedLoopOutputType.TorqueCurrentFOC;

      HOOD_TALON_CONFIG.CurrentLimits.StatorCurrentLimit = 60;
      HOOD_TALON_CONFIG.CurrentLimits.SupplyCurrentLimit = 30;

      HOOD_CONFIG.talonCANID = new CANDeviceId(17, "469CanivoreB");
      HOOD_CONFIG.fxConfig = HOOD_TALON_CONFIG;
    }

    public static final MotorIO PIVOT_MOTOR = new MotorIOTalonFX(HOOD_CONFIG);
  }
}
