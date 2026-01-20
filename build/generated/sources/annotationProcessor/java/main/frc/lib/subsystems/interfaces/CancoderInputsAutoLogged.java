package frc.lib.subsystems.interfaces;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CancoderInputsAutoLogged extends CanCoderIO.CancoderInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("AbsolutePosition", absolutePosition);
    table.put("Velocity", velocity);
  }

  @Override
  public void fromLog(LogTable table) {
    absolutePosition = table.get("AbsolutePosition", absolutePosition);
    velocity = table.get("Velocity", velocity);
  }

  public CancoderInputsAutoLogged clone() {
    CancoderInputsAutoLogged copy = new CancoderInputsAutoLogged();
    copy.absolutePosition = this.absolutePosition;
    copy.velocity = this.velocity;
    return copy;
  }
}
