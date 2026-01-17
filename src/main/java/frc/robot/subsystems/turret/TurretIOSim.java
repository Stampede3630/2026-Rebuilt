package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretMotor;
  private final DCMotorSim hoodMotor;
  // private final CANcoder cancoder;


  // turret motor

  // angle offset
  private double angleInitRad = 0.0;
  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  private final PIDController controller = new PIDController(10, 10, 10);

  public TurretIOSim() {
    // init turret motor
    turretMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 1.0),
            DCMotor.getKrakenX60Foc(1));

    // turretConfig
    //     .withMotorOutput(new MotorOutputConfigs())
    //     .withSlot0(
    //         new Slot0Configs()
    //             .withKS(10)
    //             .withKV(10)
    //             .withKA(10)
    //             .withKP(10)
    //             .withKI(10)
    //             .withKD(10)); /* set PID */
    // turretMotor.getConfigurator().apply(turretConfig);

    // init hood motor
    hoodMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.004, 10),
            DCMotor.getKrakenX60Foc(1));

    // hoodConfig.withMotorOutput(new MotorOutputConfigs());
    // hoodMotor.getConfigurator().apply(hoodConfig);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // boolean connected =
    //     BaseStatusSignal.refreshAll(
    //             turretPosition,
    //             turretVelocity,
    //             turretTorqueCurrent,
    //             turretVoltage,
    //             turretStatorCurrent,
    //             turretSupplyCurrent,
    //             turretTemp,
    //             hoodPosition,
    //             hoodVelocity,
    //             hoodTorqueCurrent,
    //             hoodVoltage,
    //             hoodStatorCurrent,
    //             hoodSupplyCurrent,
    //             hoodTemp)
    //         .isOK();

    inputs.connected = true;

    // turretMotor
    inputs.turretPosition = turretMotor.getAngularPosition().magnitude();
    // System.out.println(turretMotor.getInputVoltage());
    inputs.turretVelocity = turretMotor.getAngularVelocityRadPerSec();
    inputs.turretTorqueCurrent = turretMotor.getTorqueNewtonMeters();
    inputs.turretVoltage = turretMotor.getInputVoltage();
    inputs.turretStatorCurrent = turretMotor.getCurrentDrawAmps();
    inputs.turretSupplyCurrent = turretMotor.getCurrentDrawAmps();
    inputs.turretTemp = 1.0;

    // hoodMotor
    inputs.hoodPosition = hoodMotor.getAngularPositionRad();
    inputs.hoodVelocity = hoodMotor.getAngularVelocityRadPerSec();
    inputs.hoodTorqueCurrent = hoodMotor.getTorqueNewtonMeters();
    inputs.hoodVoltage = hoodMotor.getInputVoltage();
    inputs.hoodStatorCurrent = hoodMotor.getCurrentDrawAmps();
    inputs.hoodSupplyCurrent = hoodMotor.getCurrentDrawAmps();
    inputs.hoodTemp = 1.0;
  }

  // @Override
  // public void runWithDist(DoubleSupplier dist) {
  //     runVelocity(getValue(dist));
  // }

  /**
   * @param newAngle The angle to set the turret motors adjustment to, in radians
   */
  @Override
  public void setAngleInit(double newAngle) {
    angleInitRad = newAngle;
  }

  @Override
  public void setAngle(Angle angle) {
    // need to subtract angleInitRad here
    System.out.println("set to " + angle.magnitude());
    controller.setSetpoint(angle.magnitude());
    turretMotor.setInputVoltage(controller.calculate(turretMotor.getAngularPositionRad()));
  }

  @Override
  public void setHoodAngle(Angle angle) {
    // hoodMotor.setControl(new PositionDutyCycle(angle));
  }

  /**
   * @return The current angle of the turret, in radians
   */
  @Override
  public double getTurretAngle() {
    return turretMotor.getAngularPositionRad();
  }

  @Override
  public boolean isInitSet() {
    return initSet;
  }

  @Override
  public void updateInitSet(boolean set) {
    initSet = set;
  }

  /**
   * @return The current angle of the hood, in rotations
   */
  @Override
  public Angle getHoodAngle() {
    // return null;
    return Rotations.of(hoodMotor.getAngularPositionRotations() / Math.PI / 2);
  }

  @Override
  public void setTurretMotorControl(VoltageOut volts) {
    turretMotor.setInputVoltage(volts.Output);
  }
}
