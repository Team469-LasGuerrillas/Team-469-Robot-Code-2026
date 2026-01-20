package frc.robot.subsystems;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.LimelightHelpers;
import frc.lib.subsystems.interfaces.CanCoderIO;
import frc.lib.subsystems.interfaces.CancoderInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Exampe extends SubsystemBase {
  private final CanCoderIO canCoder;
  private final CancoderInputsAutoLogged inputs = new CancoderInputsAutoLogged();

  public Exampe(CanCoderIO canCoder) {
    this.canCoder = canCoder;
  }

  @Override
  public void periodic() {
    canCoder.readInputs(inputs);
    Logger.processInputs(getName(), inputs);

    Logger.recordOutput("Turd pose", LimelightHelpers.getCameraPose3d_RobotSpace("limelight-turd"));
  }

  public Angle getPosition() {
    return inputs.absolutePosition;
  }
}
