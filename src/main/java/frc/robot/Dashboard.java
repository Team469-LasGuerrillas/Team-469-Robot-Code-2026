package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.Elastic;
import frc.lib.utilities.field.Station;
import frc.robot.commands.AutonCommands;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Dashboard {
  private static String teleopTab = "Teleoperated";
  private static String autonTab = "Autonomous";

  public static LoggedDashboardChooser<Command> autonChooser =
      new LoggedDashboardChooser<>("Selected Auton");

  public static void setChangeTabToTeleop() {
    Elastic.selectTab(teleopTab);
  }

  public static void setChangeTabToAuton() {
    Elastic.selectTab(autonTab);
  }

  public static void configureAutonDashboard() {
    Elastic.selectTab(autonTab);

    autonChooser.addDefaultOption("Default Option", Commands.none());
    autonChooser.addOption(
        "BlueLeft",
        AutonCommands.leftPassRed(false, false));
    autonChooser.addOption(
        "BlueRight",
        AutonCommands.leftPassRed(false, true));
    autonChooser.addOption(
        "RedLeft",
        AutonCommands.leftPassRed(true, false));
    autonChooser.addOption(
        "RedRight",
        AutonCommands.leftPassRed(true, true));
  }
}
