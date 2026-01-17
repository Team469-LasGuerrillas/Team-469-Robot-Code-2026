// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.configs.ServoMotorSubsystemConfig;
import frc.lib.subsystems.implementations.MotorIOTalonFX;
import frc.lib.subsystems.implementations.VisionIOLimelight;

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

  public static class VisionC {
    public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.55;
    public static final double MIN_SINGLE_TAG_AREA = 0.33;
    public static final Angle MAX_YAW_ERROR_MT1 = Degrees.of(2.41);
    public static final Angle MAX_YAW_ERROR_MT2 = Degrees.of(1.67);
    public static final Distance MAX_FLOATING_NOCLIP = Meters.of(0.2);

    public static final VisionIOLimelight DEV_LIMELIGHT =
        VisionIOLimelight.getInstance(
            "limelight-dev",
            new Pose3d(
                Units.inchesToMeters(-4.232579),
                Units.inchesToMeters(-8.848357),
                Units.inchesToMeters(5.60048 - 0.125),
                new Rotation3d(0, Units.degreesToRadians(26), Units.degreesToRadians(15.2))));
  }

  public static class ExampeC {

    public static final ServoMotorSubsystemConfig EXAMPE_CONFIG = new ServoMotorSubsystemConfig();

    static {
      EXAMPE_CONFIG.talonCANID = new CANDeviceId(41);
    }

    public static final MotorIOTalonFX EXMAPE_MOTOR = new MotorIOTalonFX(EXAMPE_CONFIG);
  }
}
