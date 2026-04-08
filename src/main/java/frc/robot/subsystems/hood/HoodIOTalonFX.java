package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;

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

  public HoodIOTalonFX() {
    // init hood motor + signals
    hoodMotor = new TalonFX(Constants.HOOD_ID);
    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    torqueCurrent = hoodMotor.getTorqueCurrent();
    voltage = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temp = hoodMotor.getDeviceTemp();

    config.withMotorOutput(
        new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0( // TODO: tune PID
            new Slot0Configs()
                .withKS(0.0)
                .withKV(0.0)
                .withKA(0.0)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0)); /* set PID */
    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    boolean connected =
        BaseStatusSignal.refreshAll(
                position, velocity, torqueCurrent, voltage, statorCurrent, supplyCurrent, temp)
            .isOK();

    inputs.connected = connDebouncer.calculate(connected);

    // update hoodMotor
    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.voltage = voltage.getValue();
    inputs.statorCurrent = statorCurrent.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.temp = temp.getValue();
    inputs.setpoint = setpoint;
  }

  @Override
  public void stopHood() {
    hoodMotor.stopMotor();
  }
}
