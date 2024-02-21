package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    double voltage = 0.0;
    double angle = 0.0;
    double absoluteAngle = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}
  public default void setVoltage(double volts) {}
  public default double getAngle() { return 0; }

  public default void updateSim() {}

  public default void setCoast(boolean coast) {}
  public default boolean getCoast(int motor) { return false; }
}
