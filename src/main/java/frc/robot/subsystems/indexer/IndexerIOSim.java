package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim spin;
  private final DCMotorSim chute;

  // private final CANcoder cancoder;

  // turret motor

  // whether the angle offset has been set since the robot's code last booted
  //   private boolean initSet = false;

  private double chuteDutyCycle = 0.0;
  private double spinDutyCycle = 0.0;

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
    inputs.spinPosition = spin.getAngularPosition();
    inputs.spinVelocity =
        RotationsPerSecond.of(
            spin.getAngularVelocityRPM()); /* 60.0; why this */ // rotations per second
    inputs.spinTorqueCurrent = Amps.of(0.0);
    inputs.spinVoltage = Volts.of(spin.getInputVoltage());
    inputs.spinStatorCurrent = Amps.of(0.0);
    inputs.spinSupplyCurrent = Amps.of(spin.getCurrentDrawAmps());
    inputs.spinTemp = Celsius.of(0.0);

    // chute
    inputs.chutePosition = chute.getAngularPosition();
    inputs.chuteVelocity =
        RotationsPerSecond.of(chute.getAngularVelocityRPM()); /* 60.0; */ // rotations per second
    inputs.chuteTorqueCurrent = Amps.of(0.0);
    inputs.chuteVoltage = Volts.of(chute.getInputVoltage());
    inputs.chuteStatorCurrent = Amps.of(0.0);
    inputs.chuteSupplyCurrent = Amps.of(chute.getCurrentDrawAmps());
    inputs.chuteTemp = Celsius.of(0.0);

    inputs.chuteDutyCycle = chuteDutyCycle;
    inputs.spinDutyCycle = spinDutyCycle;
  }

  @Override
  public void runDutyCycleChute(double dutyCycle) {
    // TODO Auto-generated method stub
    chute.setInputVoltage(12 * dutyCycle);
    chuteDutyCycle = dutyCycle;
  }

  @Override
  public void runDutyCycleSpin(double dutyCycle) {
    // TODO Auto-generated method stub
    spin.setInputVoltage(12 * dutyCycle);
    spinDutyCycle = dutyCycle;
  }

  @Override
  public void stopChute() {
    // TODO Auto-generated method stub
    chute.setInputVoltage(0);
    chuteDutyCycle = 0.0;
  }

  @Override
  public void stopSpin() {
    // TODO Auto-generated method stub
    spin.setInputVoltage(0);
    spinDutyCycle = 0.0;
  }
}
