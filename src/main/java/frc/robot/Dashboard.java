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
    autonChooser.addOption("BlueHairpinSweep0", AutonCommands.thirdHairpinAuto(0, false));
    autonChooser.addOption("RedHairpinSweep0", AutonCommands.thirdHairpinAuto(0, true));
    autonChooser.addOption("BlueHairpinSweep2", AutonCommands.thirdHairpinAuto(2, false));
    autonChooser.addOption("RedHairpinSweep2", AutonCommands.thirdHairpinAuto(2, true));
    autonChooser.addOption("BlueHairpinSweep3", AutonCommands.thirdHairpinAuto(3, false));
    autonChooser.addOption("RedHairpinSweep3", AutonCommands.thirdHairpinAuto(3, true));
    autonChooser.addOption("BlueHairpinSweep4", AutonCommands.thirdHairpinAuto(4, false));
    autonChooser.addOption("RedHairpinSweep4", AutonCommands.thirdHairpinAuto(4, true));
    autonChooser.addOption("BlueHairpinSweep5", AutonCommands.thirdHairpinAuto(5, false));
    autonChooser.addOption("RedHairpinSweep5", AutonCommands.thirdHairpinAuto(5, true));
    autonChooser.addOption("BlueLongSweep0", AutonCommands.thirdSweepAuto(0, false));
    autonChooser.addOption("RedLongSweep0", AutonCommands.thirdSweepAuto(0, true));
    autonChooser.addOption("BlueLongSweep2", AutonCommands.thirdSweepAuto(2, false));
    autonChooser.addOption("RedLongSweep2", AutonCommands.thirdSweepAuto(2, true));
    autonChooser.addOption("BlueLongSweep3", AutonCommands.thirdSweepAuto(3, false));
    autonChooser.addOption("RedLongSweep3", AutonCommands.thirdSweepAuto(3, true));
    autonChooser.addOption("BlueLongSweep4", AutonCommands.thirdSweepAuto(4, false));
    autonChooser.addOption("RedLongSweep4", AutonCommands.thirdSweepAuto(4, true));
    autonChooser.addOption("BlueLongSweep5", AutonCommands.thirdSweepAuto(5, false));
    autonChooser.addOption("RedLongSweep5", AutonCommands.thirdSweepAuto(5, true));
    autonChooser.addOption("BlueHigh", AutonCommands.highAuto(false));
    autonChooser.addOption("RedHigh", AutonCommands.highAuto(true));
    autonChooser.addOption("Blue3RD", AutonCommands.marcusDepotAuto(false));
    autonChooser.addOption("Red3RD", AutonCommands.marcusDepotAuto(true));
    autonChooser.addOption("316style", AutonCommands.lunatecStyleAuto(true, false));

    SmartDashboard.putData("selected auton", autonChooser);
  }
}
