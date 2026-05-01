package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.Elastic;
import frc.robot.commands.AutonCommands;

public class Dashboard {
  private static String teleopTab = "Teleoperated";
  private static String autonTab = "Autonomous";

  public static SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public static void setChangeTabToTeleop() {
    Elastic.selectTab(teleopTab);
  }

  public static void setChangeTabToAuton() {
    Elastic.selectTab(autonTab);
  }

  public static void configureAutonDashboard() {
    Elastic.selectTab(autonTab);

    autonChooser.setDefaultOption("Default Option", Commands.none());
    autonChooser.addOption("BlueLeft", AutonCommands.opStyleAuto(false, false));
    autonChooser.addOption("BlueRight", AutonCommands.opStyleAuto(false, true));
    autonChooser.addOption("RedLeft", AutonCommands.opStyleAuto(true, false));
    autonChooser.addOption("RedRight", AutonCommands.opStyleAuto(true, true));
    autonChooser.addOption("BlueLongSweep", AutonCommands.thirdSweepAuto(false));
    autonChooser.addOption("RedLongSweep", AutonCommands.thirdSweepAuto(true));
    autonChooser.addOption("BlueHigh", AutonCommands.highAuto(false));
    autonChooser.addOption("RedHigh", AutonCommands.highAuto(true));
    autonChooser.addOption("Blue3RD", AutonCommands.marcusDepotAuto(false));
    autonChooser.addOption("Red3RD", AutonCommands.marcusDepotAuto(true));

    SmartDashboard.putData("selected auton", autonChooser);
  }
}
