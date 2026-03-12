// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.lib.subsystems.configs.ServoMotorSubsystemWithFollowersConfig;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.VisionIO;
import frc.lib.subsystems.interfaces.VisionIO.PoseObservation;
import frc.robot.commands.AutonCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeederCommands;
import frc.robot.commands.HoodCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.SpindexerCommands;
import frc.robot.commands.TurretCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.FiducialVision;
import frc.robot.subsystems.vision.util.FiducialFilters.FiducialModifications;
import frc.robot.util.HubShiftUtil;
import frc.robot.util.ShootTarget;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.function.Function;
import java.util.function.UnaryOperator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final FiducialVision limelightRight;
  private final FiducialVision limelightLeft;
  private final FiducialVision limelightClimb;
  public final FiducialVision limelightTurd;
  public final Turret exampe;
  public final Intake intake;
  public final Spindexer spindexer;
  public final Shooter shooter;
  public final Feeder feeder;
  public final Hood hood;
  public final Climb climb;

  // Controller
  private final CommandXboxController marcus = new CommandXboxController(0);
  private final CommandXboxController kyle = new CommandXboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            Drive.getInstance(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        limelightRight =
            new FiducialVision(
                Constants.VisionC.LIMELIGHT_RIGHT,
                new ArrayList<Function<PoseObservation, Boolean>>(),
                Constants.VisionC.LL3G_MODIFICATIONS);

        limelightLeft =
            new FiducialVision(
                Constants.VisionC.LIMELIGHT_LEFT,
                new ArrayList<Function<PoseObservation, Boolean>>(),
                new ArrayList<UnaryOperator<FiducialModifications>>());

        limelightClimb =
            new FiducialVision(
                Constants.VisionC.LIMELIGHT_CLIMB,
                new ArrayList<Function<PoseObservation, Boolean>>(),
                new ArrayList<UnaryOperator<FiducialModifications>>());

        limelightTurd =
            new FiducialVision(
                Constants.VisionC.TURD_LIMELIGHT,
                new ArrayList<Function<PoseObservation, Boolean>>(),
                Constants.VisionC.TURRET_MODIFICATIONS);

        exampe =
            Turret.createInstance(
                Constants.TurretC.motah, Constants.TurretC.coderA, Constants.TurretC.coderB);

        intake =
            Intake.createinstance(
                Constants.IntakeC.ROLLER_MOTOR,
                Constants.IntakeC.PIVOT_MOTOR,
                Constants.IntakeC.coder);

        spindexer = Spindexer.createinstance(Constants.SpindexerC.SPINDEXER_MOTOR);

        shooter =
            Shooter.createinstance(
                Constants.LauncherC.LAUNCHER_CONFIG,
                Constants.LauncherC.LAUNCHER_MOTOR,
                Constants.LauncherC.FOLLOWER_MOTORS);

        feeder = Feeder.createinstance(Constants.FeederC.FEEDER_MOTOR);

        hood = Hood.createinstance(Constants.HoodC.PIVOT_MOTOR);

        climb = Climb.createInstance(Constants.ClimbC.CLIMB_MOTOR);

        break;

        /*
         * case SIM:
         * // Sim robot, instantiate physics sim IO implementations
         * drive = Drive.getInstance(
         * new GyroIO() {
         * },
         * new ModuleIOSim(TunerConstants.FrontLeft),
         * new ModuleIOSim(TunerConstants.FrontRight),
         * new ModuleIOSim(TunerConstants.BackLeft),
         * new ModuleIOSim(TunerConstants.BackRight));
         * break;
         */

      default:
        // Replayed robot, disable IO implementations
        drive =
            Drive.getInstance(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        limelightRight = new FiducialVision(new VisionIO() {}, null, null);

        limelightLeft = new FiducialVision(new VisionIO() {}, null, null);

        limelightClimb = new FiducialVision(new VisionIO() {}, null, null);

        limelightTurd = new FiducialVision(new VisionIO() {}, null, null);

        exampe = Turret.createInstance(new MotorIO() {}, new CanCoderIO() {}, new CanCoderIO() {});

        intake = Intake.createinstance(new MotorIO() {}, new MotorIO() {}, new CanCoderIO() {});

        spindexer = Spindexer.createinstance(new MotorIO() {});

        shooter =
            Shooter.createinstance(
                new ServoMotorSubsystemWithFollowersConfig() {}, new MotorIO() {}, null);

        feeder = Feeder.createinstance(new MotorIO() {});

        hood = Hood.createinstance(new MotorIO() {});

        climb = Climb.createInstance(new MotorIO() {});

        break;
    }

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(HubShiftUtil::initialize));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(HubShiftUtil::initialize).ignoringDisable(true));

    // Configure the button
    configurePPNamedCommands();
    configureDefaultCommands();
    configureButtonBindings();

    Dashboard.configureAutonDashboard();
  }

  private void configurePPNamedCommands() {
    NamedCommands.registerCommand("IntakeStart", IntakeCommands.deployAndRun());
  }

  private void configureDefaultCommands() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -marcus.getLeftY(),
            () -> -marcus.getLeftX(),
            () -> -marcus.getRightX()));

    HashSet<Subsystem> turretList = new HashSet<Subsystem>();
    turretList.add(exampe);

    // exampe.setDefaultCommand(TurretCommands.targetAngle(Rotations.of(0)));

    exampe.setDefaultCommand(
        Commands.defer(
            () -> TurretCommands.targetPoint(ShootTarget::getTranslationToTarget), turretList));

    intake.setDefaultCommand(IntakeCommands.stow());

    spindexer.setDefaultCommand(SpindexerCommands.agitate());

    shooter.setDefaultCommand(ShooterCommands.idleCommand());

    feeder.setDefaultCommand(FeederCommands.idleCommand());

    hood.setDefaultCommand(HoodCommands.stowHood());

    climb.setDefaultCommand(ClimbCommands.setClimbOff());
  }

  private void configureButtonBindings() {
    HashSet<Subsystem> driveList = new HashSet<Subsystem>();
    driveList.add(Drive.getInstance());

    // Lock to 0° when A button is held
    marcus.povUp().whileTrue(ClimbCommands.setClimbUp());
    marcus.povDown().whileTrue(ClimbCommands.setClimbDown());
    marcus.povUp().onTrue(TurretCommands.targetAngle(Rotations.of(-0.23)));

    // Switch to X pattern when X button is pressed
    marcus.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    marcus.b().whileTrue(AutonCommands.leftPassRed());

    marcus.leftBumper().toggleOnTrue(IntakeCommands.deployAndRun());

    marcus.rightBumper().toggleOnTrue(CommandFactory.feedOrScore());

    kyle.rightBumper().whileTrue(CommandFactory.unJam());
    kyle.leftTrigger().whileTrue(CommandFactory.outTake());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return Dashboard.autonChooser.get();
  }
}
