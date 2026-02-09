package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import frc.lib.subsystems.interfaces.MotorIO;
import frc.lib.subsystems.interfaces.MotorInputsAutoLogged;

public class Shooter extends SubsystemBase {
    
    private static Shooter instance;

    private final MotorIO leadFlywheel;

    private final Follower followerControl1 = new Follower(0, MotorAlignmentValue.Aligned);
    private final Follower followerControl2 = new Follower(0, MotorAlignmentValue.Opposed);
    private final Follower followerControl3 = new Follower(0, MotorAlignmentValue.Opposed);
  
  private final MotorInputsAutoLogged talonInputs = new MotorInputsAutoLogged();
  private final CancoderInputsAutoLogged ccInputs = new CancoderInputsAutoLogged();

  private final CanCoderIO canCoder;

  private double requestedDutycycle = 0;


  public Shooter createinstance() {
    return instance;
  }

  public static Shooter Getinstance() {
    return instance;
  }

  private Shooter(MotorIO shooterMotor, CanCoderIO canCoder) {
    this.leadFlywheel = shooterMotor;
    this.canCoder = canCoder;

    shooterMotor.setEnableSoftLimits(true, true);
  }

  public void setDutyCycle(double dutyCycle) {
    requestedDutycycle = dutyCycle;
    leadFlywheel.setOpenLoopDutyCycle(requestedDutycycle);
  }

  

  @Override
  public void periodic() {
    leadFlywheel.readInputs(talonInputs);
    Logger.processInputs(getName() + "flywheelLead", talonInputs);
    canCoder.readInputs(ccInputs);
    Logger.processInputs(getName() + "CanCoder", ccInputs);
  }

}

