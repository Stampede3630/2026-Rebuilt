package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class TurretIOSim implements TurretIO {
  private boolean turretActive = false;
  private LoggedNetworkBoolean disablePID =
      new LoggedNetworkBoolean("Sim/disabledTurretPID", false);

  private final DCMotorSim turretMotor;
  // private final CANcoder cancoder;

  // turret motor

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  private final PIDController controller = new PIDController(1.4, 0.01, 0.2);

  public TurretIOSim() {
    // init turret motor
    turretMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Update simulation state
    // turretMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
    // hoodMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
    if (turretActive)
      turretMotor.setInputVoltage(controller.calculate(turretMotor.getAngularPositionRad()));
    else {
      controller.reset();
      turretMotor.setInputVoltage(0);
    }
    turretMotor.update(0.02);

    inputs.connected = true;

    // turretMotor
    inputs.position =
        turretMotor.getAngularPositionRad() / Math.PI * 180 /* convert to degrees cuz why not */;
    // System.out.println(turretMotor.getInputVoltage());
    inputs.velocity = turretMotor.getAngularVelocityRadPerSec();
    inputs.torqueCurrent = turretMotor.getTorqueNewtonMeters();
    inputs.voltage = turretMotor.getInputVoltage();
    inputs.statorCurrent = turretMotor.getCurrentDrawAmps();
    inputs.supplyCurrent = turretMotor.getCurrentDrawAmps();
    inputs.temp = 1.0;
    inputs.setpoint = controller.getSetpoint() / Math.PI * 180 /* convert to degrees cuz why not */;
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  /**
   * @param newAngle The angle to set the turret motors adjustment to, in radians
   */
  @Override
  public void resetAnglePos(Angle newAngle) {
    turretMotor.setAngle(newAngle.in(Radians));
  }

  @Override
  public void setTurretAngle(Angle angle) {
    // need to subtract angleInitRad here
    // System.out.println("set to " + angle.magnitude());
    if (!disablePID.getAsBoolean()) {
      turretActive = true;
      controller.setSetpoint(angle.magnitude());
    } else {
      turretMotor.setAngle(angle.magnitude());
    }
  }

  /**
   * @return The current angle of the turret, in radians
   */
  @Override
  public Angle getTurretAngle() {
    return turretMotor.getAngularPosition();
  }

  @Override
  public boolean isInitSet() {
    return initSet;
  }

  @Override
  public void updateInitSet(boolean set) {
    initSet = set;
  }

  @Override
  public void setTurretMotorControl(VoltageOut volts) {
    turretMotor.setInputVoltage(volts.Output);
  }

  @Override
  public void stopTurret() {
    turretActive = false;
  }
}
