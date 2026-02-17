package frc.robot.subsystems.hood;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class HoodIOSim implements HoodIO {
  private boolean hoodActive = false;
  private LoggedNetworkBoolean disablePID = new LoggedNetworkBoolean("Sim/disableHoodPID", false);

  private final DCMotorSim hoodMotor;

  private final PIDController controller = new PIDController(4.0, 0.01, 0.2);

  public HoodIOSim() {
    // init hood motor
    hoodMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 10),
            DCMotor.getKrakenX60Foc(1));

    controller.setSetpoint(0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    // Update simulation state
    // turretMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    // hoodMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    if (hoodActive)
      hoodMotor.setInputVoltage(controller.calculate(hoodMotor.getAngularPositionRad()));
    else {
      controller.reset();
      hoodMotor.setInputVoltage(0);
    }
    hoodMotor.update(0.02);

    // inputs.connected = true;

    // // hoodMotor
    // inputs.position = hoodMotor.getAngularPositionRad() / Math.PI * 180;
    // inputs.velocity = hoodMotor.getAngularVelocityRadPerSec();
    // inputs.torqueCurrent = hoodMotor.getTorqueNewtonMeters();
    // inputs.voltage = hoodMotor.getInputVoltage();
    // inputs.statorCurrent = hoodMotor.getCurrentDrawAmps();
    // inputs.supplyCurrent = hoodMotor.getCurrentDrawAmps();
    // inputs.temp = 1.0;
    // inputs.setpoint = controller.getSetpoint() / Math.PI * 180 /* convert to degrees */;
  }

  @Override
  public void stopHood() {
    hoodActive = false;
    hoodMotor.setAngularVelocity(0);
  }
}
