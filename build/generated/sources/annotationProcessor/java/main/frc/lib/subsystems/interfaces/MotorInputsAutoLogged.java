package frc.lib.subsystems.interfaces;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class MotorInputsAutoLogged extends MotorIO.MotorInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("MotorPosition", motorPosition);
    table.put("RawMotorPoAngle", rawMotorPosition);
    table.put("MotorVelocity", motorVelocity);
    table.put("MotorAngularAcceleration", motorAngularAcceleration);
    table.put("TargetPosition", targetPosition);
    table.put("TargetVelocity", targetVelocity);
    table.put("AppliedVolts", appliedVolts);
    table.put("SupplyCurrent", supplyCurrent);
    table.put("StatorCurrent", statorCurrent);
  }

  @Override
  public void fromLog(LogTable table) {
    motorPosition = table.get("MotorPosition", motorPosition);
    rawMotorPosition = table.get("RawMotorPoAngle", rawMotorPosition);
    motorVelocity = table.get("MotorVelocity", motorVelocity);
    motorAngularAcceleration = table.get("MotorAngularAcceleration", motorAngularAcceleration);
    targetPosition = table.get("TargetPosition", targetPosition);
    targetVelocity = table.get("TargetVelocity", targetVelocity);
    appliedVolts = table.get("AppliedVolts", appliedVolts);
    supplyCurrent = table.get("SupplyCurrent", supplyCurrent);
    statorCurrent = table.get("StatorCurrent", statorCurrent);
  }

  public MotorInputsAutoLogged clone() {
    MotorInputsAutoLogged copy = new MotorInputsAutoLogged();
    copy.motorPosition = this.motorPosition;
    copy.rawMotorPosition = this.rawMotorPosition;
    copy.motorVelocity = this.motorVelocity;
    copy.motorAngularAcceleration = this.motorAngularAcceleration;
    copy.targetPosition = this.targetPosition;
    copy.targetVelocity = this.targetVelocity;
    copy.appliedVolts = this.appliedVolts;
    copy.supplyCurrent = this.supplyCurrent;
    copy.statorCurrent = this.statorCurrent;
    return copy;
  }
}
