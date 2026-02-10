package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.drivers.Elastic;

public class Dashboard {
  private static String teleopTab = "Teleoperated";
  private static String autonTab = "Autonomous";

  private static SendableChooser<Command> autonChooser = new SendableChooser<Command>();

  public static void setChangeTabToTeleop() {
    Elastic.selectTab(teleopTab);
  }

  public static void setChangeTabToAuton() {
    Elastic.selectTab(autonTab);
  }

  public static void configureAutonDashboard() {
    ShuffleboardTab autonTab = Shuffleboard.getTab("Autonomous");

    autonChooser.addOption("Null auton", Commands.none());

    autonTab
        .add("Auton Chooser", autonChooser)
        .withSize(2, 5)
        .withPosition(13, 0);

  }
}
