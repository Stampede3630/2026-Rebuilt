package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  // default void runVelocity(double vel) {}

  default void updateInputs(IndexerIOInputs inputs) {}

  // default boolean setCoastMode(boolean enabled) {
  //   return true;
  // }

  default void runDutyCycleChute(double dutyCycle) {}

  default void stopChute() {}

  default void runDutyCycleSpin(double dutyCycle) {}

  default void stopSpin() {}

  // default double getShooterSpeed() {
  //   return 0.0;
  // }

  // default void setShooterMotorsControl(VoltageOut volts) {}

  @AutoLog
  class IndexerIOInputs {
    public boolean spinConnected = false;
    public boolean chuteConnected = false;

    // spin motor
    public Angle spinPosition = Rotations.of(0.0);
    public AngularVelocity spinVelocity = RotationsPerSecond.of(0.0);
    public Current spinTorqueCurrent = Amps.of(0.0);
    public Voltage spinVoltage = Volts.of(0.0);
    public Current spinStatorCurrent = Amps.of(0.0);
    public Current spinSupplyCurrent = Amps.of(0.0);
    public Temperature spinTemp = Celsius.of(0.0);

    // chute motor
    public Angle chutePosition = Rotations.of(0.0);
    public AngularVelocity chuteVelocity = RotationsPerSecond.of(0.0);
    public Current chuteTorqueCurrent = Amps.of(0.0);
    public Voltage chuteVoltage = Volts.of(0.0);
    public Current chuteStatorCurrent = Amps.of(0.0);
    public Current chuteSupplyCurrent = Amps.of(0.0);
    public Temperature chuteTemp = Celsius.of(0.0);

    public double spinDutyCycle = 0.0;
    public double chuteDutyCycle = 0.0;
  }
}
