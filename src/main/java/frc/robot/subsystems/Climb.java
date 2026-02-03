package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private static Climb instance;

    private final MotorIO climbMotor;
    private final MotorIO climbFollowerMotor;
    private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
    private final CanCoderIO canCoder;
    private final CancoderInputsAutoLogged ccInputs = new CancoderInputsAutoLogged();

    private double requestedDutyCycle = 0;

    public Climb createInstance() {
        return instance;
    }

    public static Climb GetInstance() {
        return instance;
    }

    private Climb(MotorIO climbMotor, MotorIO climbFollowerMotor, CanCoderIO canCoder) {
        this.climbMotor = climbMotor;
        this.climbFollowerMotor = climbFollowerMotor;
        this.canCoder = canCoder;

        climbMotor.setEnableSoftLimits(true, true);
        climbFollowerMotor.setEnableSoftLimits(true, true);
    }

    public void setDutyCycle (double dutyCycle) {
        requestedDutyCycle = dutyCycle;
        climbMotor.setOpenLoopDutyCycle(dutyCycle);
        climbFollowerMotor.setOpenLoopDutyCycle(dutyCycle);
    }

     @Override
  public void periodic() {
    climbMotor.readInputs(talonInputs);
    Logger.processInputs(getName() + "Climb Motor", talonInputs);
    climbFollowerMotor.readInputs(talonInputs);
    Logger.processInputs(getName() + "Follower Motor", talonInputs);
    canCoder.readInputs(ccInputs);
    Logger.processInputs(getName() + "CanCoder", ccInputs);
  }
}
