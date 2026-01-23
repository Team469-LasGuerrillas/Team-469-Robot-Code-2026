// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.configs.CanCoderConfig;
import frc.lib.subsystems.implementations.CanCoderIOCanCoder;
import frc.lib.subsystems.implementations.VisionIOLimelight;
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

  public static class Field {
    public static final AprilTagFieldLayout WELDED_FIELD =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Distance MAX_FIELD_X = Meters.of(WELDED_FIELD.getFieldLength());
    public static final Distance MAX_FIELD_Y = Meters.of(WELDED_FIELD.getFieldWidth());
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
                0.060712,
                0.173054,
                0.299212 - Units.inchesToMeters(0.125),
                new Rotation3d(
                    0, Units.degreesToRadians(28), Units.degreesToRadians(180 - 20.220574))));
  }

  public static class ExampeC {

    public static final CanCoderConfig EXAMPE_CONFIG = new CanCoderConfig();

    static {
      EXAMPE_CONFIG.CANID = new CANDeviceId(5);
      EXAMPE_CONFIG.config = new CANcoderConfiguration();
      EXAMPE_CONFIG.config.MagnetSensor.MagnetOffset = -0.36523433383028964;
      EXAMPE_CONFIG.config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    }

    public static final CanCoderIOCanCoder coder = new CanCoderIOCanCoder(EXAMPE_CONFIG);
  }
}
