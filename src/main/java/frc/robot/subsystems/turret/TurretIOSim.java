package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.logging.Logger;

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
        Degrees.of(
            turretMotor.getAngularPositionRad()
                / Math.PI
                * 180) /* convert to degrees cuz why not */;
    // System.out.println(turretMotor.getInputVoltage());
    inputs.velocity = turretMotor.getAngularVelocity();
    inputs.torqueCurrent = Amps.of(turretMotor.getTorqueNewtonMeters());
    inputs.voltage = Volts.of(turretMotor.getInputVoltage());
    inputs.statorCurrent = Amps.of(turretMotor.getCurrentDrawAmps());
    inputs.supplyCurrent = Amps.of(turretMotor.getCurrentDrawAmps());
    inputs.temp = Celsius.of(1.0);
    inputs.setpoint =
        Degrees.of(controller.getSetpoint() / Math.PI * 180); /* convert to degrees cuz why not */
    ;
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

  private final double LEFT_LIMIT = -170.0 / 360.0; // rotations was -0.15; used to -0.75
  private final double RIGHT_LIMIT = 0.625; // rotations was 1.11

  @Override
  public void setTurretAngle(Angle angle) {
    // need to subtract angleInitRad here
    // System.out.println("set to " + angle.magnitude());
    Angle leftAngle = angle.minus(Degrees.of(360)); // check near left rotation
    Angle rightAngle = angle.plus(Degrees.of(360));
    Angle currentAngle = turretMotor.getAngularPosition();
    // find if leftAngle or angle is closer to currentAngle
    if (currentAngle.minus(leftAngle).abs(Radians) < currentAngle.minus(angle).abs(Radians)
        && currentAngle.minus(leftAngle).abs(Radians) < currentAngle.minus(rightAngle).abs(Radians)
        && leftAngle.gt(Rotations.of(LEFT_LIMIT))) {
      // if leftAngle is closer
      angle = leftAngle;
      org.littletonrobotics.junction.Logger.recordOutput("TurretStuff","L");
    } else if (currentAngle.minus(rightAngle).abs(Radians) < currentAngle.minus(angle).abs(Radians)
        && currentAngle.minus(rightAngle).abs(Radians) < currentAngle.minus(leftAngle).abs(Radians)
        && rightAngle.lt(Rotations.of(RIGHT_LIMIT))) {
      // if rightAngle is closer
      angle = rightAngle;
      org.littletonrobotics.junction.Logger.recordOutput("TurretStuff","R");

    } else { // middle
      if (angle.lt(Rotations.of(LEFT_LIMIT))) {
        angle = rightAngle;
      org.littletonrobotics.junction.Logger.recordOutput("TurretStuff","R");

      } else if (angle.gt(Rotations.of(RIGHT_LIMIT))) {
        angle = rightAngle;
      org.littletonrobotics.junction.Logger.recordOutput("TurretStuff","L");
      } else {
      org.littletonrobotics.junction.Logger.recordOutput("TurretStuff","M");
      }
    }
    if (!disablePID.getAsBoolean()) {
      turretActive = true;
      controller.setSetpoint(angle.magnitude());
    } else {
      turretMotor.setAngle(angle.magnitude());
    }
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
