package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class IndexerIOSim implements IndexerIO {
  private boolean turretActive = false;
  private LoggedNetworkBoolean disablePID =
      new LoggedNetworkBoolean("Sim/disabledIndexerPID", false);

  private final DCMotorSim spin;
  private final DCMotorSim chute;

  // private final CANcoder cancoder;

  // turret motor

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  public IndexerIOSim() {
    // init turret motor
    spin =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX60Foc(1));
    chute =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // Update simulation state
    // indexerMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    // hoodMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    spin.update(0.02);
    chute.update(0.02);

    // spin
    inputs.spinPosition = spin.getAngularPositionRotations();
    inputs.spinVelocity = spin.getAngularVelocityRPM() / 60.0; // rotations per second
    inputs.spinTorqueCurrent = 0;
    inputs.spinVoltage = spin.getInputVoltage();
    inputs.spinStatorCurrent = 0;
    inputs.spinSupplyCurrent = spin.getCurrentDrawAmps();
    inputs.spinTemp = 0;

    // chute
    inputs.chutePosition = chute.getAngularPositionRotations();
    inputs.chuteVelocity = chute.getAngularVelocityRPM() / 60.0; // rotations per second
    inputs.chuteTorqueCurrent = 0;
    inputs.chuteVoltage = chute.getInputVoltage();
    inputs.chuteStatorCurrent = 0;
    inputs.chuteSupplyCurrent = chute.getCurrentDrawAmps();
    inputs.chuteTemp = 0;
  }

  @Override
  public void runDutyCycleChute(double dutyCycle) {
    // TODO Auto-generated method stub
    chute.setInputVoltage(12 * dutyCycle);
  }

  @Override
  public void runDutyCycleSpin(double dutyCycle) {
    // TODO Auto-generated method stub
    spin.setInputVoltage(12 * dutyCycle);
  }

  @Override
  public void stopChute() {
    // TODO Auto-generated method stub
    chute.setInputVoltage(0);
  }

  @Override
  public void stopSpin() {
    // TODO Auto-generated method stub
    spin.setInputVoltage(0);
  }
}
