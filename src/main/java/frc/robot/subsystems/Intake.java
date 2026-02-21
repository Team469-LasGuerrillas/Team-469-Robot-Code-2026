package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Intake extends SubsystemBase {
  private static Intake instance;

  @AutoLogOutput(key = "Intake/Visualization")
  private LoggedMechanism2d intakeMechanism = new LoggedMechanism2d(0, 0);

  private final MotorIO rollerMotor;
  private final MotorIO pivotMotor;
  private final MotorInputsAutoLogged rollerInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged pivotInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  private Angle requestedAngle = Units.Degrees.of(0);

  public Intake createinstance(MotorIO rollerMotor, MotorIO pivotMotor) {
    instance = new Intake(rollerMotor, pivotMotor);
    return instance;
  }

  public static Intake Getinstance() {
    return instance;
  }

  private Intake(MotorIO rollerMotor, MotorIO pivotMotor) {
    this.rollerMotor = rollerMotor;
    this.pivotMotor = pivotMotor;

    pivotMotor.setEnableSoftLimits(true, true);
  }

  @Override
  public void periodic() {
    rollerMotor.readInputs(rollerInputs);
    Logger.processInputs(getName() + "Roller Motor", rollerInputs);
    pivotMotor.readInputs(pivotInputs);
    Logger.processInputs(getName() + "Pivot Motor", pivotInputs);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    rollerMotor.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public void setTargetAngle(Angle newAngleRequest) {
    requestedAngle = newAngleRequest;
    pivotMotor.setMagicalPositionSetpoint(
        newAngleRequest,
        RotationsPerSecond.of(1),
        RotationsPerSecondPerSecond.of(1),
        0,
        calcFF(newAngleRequest));
  }

  private double calcFF(Angle pivotAngle) {
    return 0;
  }
}
