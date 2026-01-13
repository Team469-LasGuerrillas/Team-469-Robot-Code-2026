package frc.lib.subsystems.interfaces;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public interface MotorIO {
    
    @AutoLog
    public class MotorInputs {
        public Angle motorPosition = Angle.ofBaseUnits(0, null);
        public Angle rawMotorPoAngle = Angle.ofBaseUnits(0, null);
        public AngularVelocity motorVelocity = AngularVelocity.ofBaseUnits(0, null);
        public AngularAcceleration motorAngularAcceleration = AngularAcceleration.ofBaseUnits(0, null);

        public Angle targetPosition = Angle.ofBaseUnits(0, null);
        public AngularVelocity targetVelocity = AngularVelocity.ofBaseUnits(0, null);

        public Voltage appliedVolts = Voltage.ofBaseUnits(0, null);
        public Current supplyCurrent = Current.ofBaseUnits(0, null);
        public Current statorCurrent = Current.ofBaseUnits(0, null);
    }

    default void readInputs(MotorInputsAutoLogged inputs) {}

    default void setOpenLoopDutyCycle(double dutyCycle) {}

    default void setPositionSetpoint(Angle position, Voltage ff) {}

    default void setMagicalPositionSetpoint(Angle position, AngularVelocity velocity, AngularAcceleration acceleration, int slot, Voltage ff) {}

    default void setVelocitySetpiont(AngularVelocity velocity) {}

    default void setMagicalVelcoitySetpoint(AngularVelocity velocity, AngularAcceleration acceleration, int slot) {}

    default void setEnableSoftLimits(boolean forward, boolean reverse) {}

    default void setMotionMagicConfig(MotionMagicConfigs config) {}

    default void setZero() {
        setCurrentPosition(Angle.ofBaseUnits(0, null));
    }

    default void setCurrentPosition(Angle position) {}
}
