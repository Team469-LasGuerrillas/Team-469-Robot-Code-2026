// package frc.robot.util;

// import org.littletonrobotics.junction.Logger;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.lib.drivers.Elastic;

// public class DashboardVisualization {

//     private static Timer autonTimer; // How have we been in auto?
//     private static Timer teleopTimer; // How long have we been in teleop?
//     private static Timer nextGoalSwapTimer; // How long the current goal has been active?
//     private static Boolean didWeWinAuton = false; // true = yes
//     private static Boolean initiated = false; // true = yes

//     // "hello world"
//     public static double getTimeRemainingInAuton() {
//         return 20 - autonTimer.get();
//     }

//     // Call me once ever rio loop please!
//     public static void periodic() {
//         if (!initiated) {
//             // run init routinen

//             autonTimer = new Timer();
//             teleopTimer = new Timer();
//             nextGoalSwapTimer = new Timer();
//             initiated = true;
//         }

//     }

//     // Call me once when auton begins!
//     public static void autonStart() {

//     }

//     //

// }
