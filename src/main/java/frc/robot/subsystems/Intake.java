package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import frc.robot.Constants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;

public class Intake extends SubsystemBase {
  private static Intake instance;

  @AutoLogOutput(key = "Intake/Visualization")
  private LoggedMechanism2d intakeMechanism = new LoggedMechanism2d(0, 0);

  private final MotorIO rollerMotor;
  private final MotorIO pivotMotor;
  private final CanCoderIO coder;
  private final CancoderInputsAutoLogged coderInputs = new CancoderInputsAutoLogged();
  private final MotorInputsAutoLogged rollerInputs = new MotorInputsAutoLogged();
  private final MotorInputsAutoLogged pivotInputs = new MotorInputsAutoLogged();

  private double requestedDutycycle = 0;

  private Angle requestedAngle = Constants.IntakeC.PIVOT_RAISED;

  public static Intake createinstance(MotorIO rollerMotor, MotorIO pivotMotor, CanCoderIO coder) {
    instance = new Intake(rollerMotor, pivotMotor, coder);
    return instance;
  }

  public static Intake getInstance() {
    return instance;
  }

  private Intake(MotorIO rollerMotor, MotorIO pivotMotor, CanCoderIO coder) {
    this.rollerMotor = rollerMotor;
    this.pivotMotor = pivotMotor;
    this.coder = coder;

    pivotMotor.setEnableSoftLimits(true, true);
  }

  @Override
  public void periodic() {
    rollerMotor.readInputs(rollerInputs);
    // Logger.processInputs(getName() + "Roller Motor", rollerInputs);
    pivotMotor.readInputs(pivotInputs);
    // Logger.processInputs(getName() + "Pivot Motor", pivotInputs);
    coder.readInputs(coderInputs);
    // Logger.processInputs(getName() + "Cancoder", coderInputs);

    pivotMotor.setMagicalPositionSetpoint(
        requestedAngle,
        RotationsPerSecond.of(9999),
        RotationsPerSecondPerSecond.of(9999),
        0,
        calcFF(requestedAngle));
  }

  @AutoLogOutput(key = "Intake/TargetAngle")
  public Angle getTargetAngle() {
    return requestedAngle;
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    rollerMotor.setOpenLoopDutyCycle(requestedDutycycle);
  }

  public void setTargetAngle(Angle newAngleRequest) {
    requestedAngle = newAngleRequest;
  }

  private double calcFF(Angle pivotAngle) {
    return 0;
  }
}
