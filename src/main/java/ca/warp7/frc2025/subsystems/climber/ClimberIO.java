package ca.warp7.frc2025.subsystems.climber;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public boolean motorConnected = false;

        public Rotation2d pivotPositionRads = Rotation2d.kZero;
        public Rotation2d pivotVelocityRadsPerSecond = Rotation2d.kZero;
        public double pivotVoltage = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempC = 0.0;
    }

    public default void updateInputs(final ClimberIOInputsAutoLogged inputs) {}

    public default void setPivotVoltage(final double volts) {}

    public default void setServoPosition(final double position) {}

    public default void setPivotSpeed(final double speed) {}

    public default void setPivotPosition(final Rotation2d position) {}

    public default void setControlConstants(double kG, double kS, double kV, double kA, double kP, double kD) {}

    public default void setPD(double kP, double kD) {
        setControlConstants(0, 0, 0, 0, kP, kD);
    }

    public default void stop() {
        setPivotVoltage(0);
    }
}
