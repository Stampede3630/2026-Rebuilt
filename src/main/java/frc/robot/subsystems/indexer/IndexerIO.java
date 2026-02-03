package frc.robot.subsystems.indexer;

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
    public boolean connected = false;

    // spin motor
    public double spinPosition = 0.0;
    public double spinVelocity = 0.0;
    public double spinTorqueCurrent = 0.0;
    public double spinVoltage = 0.0;
    public double spinStatorCurrent = 0.0;
    public double spinSupplyCurrent = 0.0;
    public double spinTemp = 0.0;

    // chute motor
    public double chutePosition = 0.0;
    public double chuteVelocity = 0.0;
    public double chuteTorqueCurrent = 0.0;
    public double chuteVoltage = 0.0;
    public double chuteStatorCurrent = 0.0;
    public double chuteSupplyCurrent = 0.0;
    public double chuteTemp = 0.0;
  }
}
