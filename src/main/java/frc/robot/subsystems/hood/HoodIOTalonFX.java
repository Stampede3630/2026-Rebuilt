package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  // private final CANcoder cancoder;

  private final Debouncer connDebouncer = new Debouncer(0.5);

  // hood motor
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Voltage> voltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Temperature> temp;
  private Angle setpoint = Radians.of(0);

  // whether the angle offset has been set since the robot's code last booted
  private boolean initSet = false;

  public HoodIOTalonFX() {
    // init hood motor
    hoodMotor = new TalonFX(Constants.HOOD_ID);
    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    voltage = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temp = hoodMotor.getDeviceTemp();

    config
        .withMotorOutput(new MotorOutputConfigs())
        .withSlot0(
            new Slot0Configs()
                .withKS(1)
                .withKV(1)
                .withKA(1)
                .withKP(1.4)
                .withKI(0.01)
                .withKD(0.2)); /* set PID */
    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, torqueCurrent, voltage, statorCurrent, supplyCurrent, temp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // hoodMotor
    inputs.position = position.getValueAsDouble();
    inputs.velocity = velocity.getValueAsDouble();
    inputs.torqueCurrent = torqueCurrent.getValueAsDouble();
    inputs.voltage = voltage.getValueAsDouble();
    inputs.statorCurrent = statorCurrent.getValueAsDouble();
    inputs.supplyCurrent = supplyCurrent.getValueAsDouble();
    inputs.temp = temp.getValueAsDouble();
    inputs.setpoint = setpoint.magnitude();
  }

  @Override
  public void setHoodAngle(Angle angle) {
    setpoint = angle;
    hoodMotor.setControl(new PositionTorqueCurrentFOC(angle));
  }

  /**
   * @return The current angle of the hood, in rotations NOTE: need to initialize at a specific
   *     angle
   */
  @Override
  public Angle getHoodAngle() {
    return hoodMotor.getPosition().getValue();
  }

  @Override
  public void runHood(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void stopHood() {
    hoodMotor.stopMotor();
  }
}
