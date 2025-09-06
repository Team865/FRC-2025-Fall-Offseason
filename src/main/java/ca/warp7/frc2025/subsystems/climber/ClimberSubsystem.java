package ca.warp7.frc2025.subsystems.climber;

import ca.warp7.frc2025.Constants.Climber;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs;
    public Rotation2d goal = Rotation2d.kZero;

    public ClimberSubsystem(ClimberIO io) {
        this.io = io;
        inputs = new ClimberIOInputsAutoLogged();
        io.setPD(Climber.kPNormal, Climber.kDNormal);
    }

    // @AutoLogOutput(key = "Climber/goal")
    // public Command setGoal(Rotation2d goal) {
    //     return this.runOnce(() -> io.setPivotPosition(goal));
    // }

    public Command setPivotVoltage(double volts) {
        return this.runOnce(() -> io.setPivotVoltage(volts));
    }

    public Command setPivotServoPosition(double position) {
        return this.runOnce(() -> io.setServoPosition(position));
    }

    public Command setPivotSpeed(double speed) {
        return this.runOnce(() -> io.setPivotSpeed(speed));
    }

    public Command setPivotPosition(Rotation2d position) {
        return this.runOnce(() -> goal = position);
    }

    public Trigger atSetpointTrigger() {
        return new Trigger(() -> MathUtil.isNear(goal.getDegrees(), inputs.pivotPositionRads.getDegrees(), 0.5));
    }

    public Command setNormalGains() {
        return runOnce(() -> io.setPD(Climber.kPNormal, Climber.kDNormal));
    }

    public Command setClimbGains() {
        return runOnce(() -> io.setPD(Climber.kPClimbing, Climber.kDClimbing));
    }

    public Command down() {
        return setPivotServoPosition(0)
                .andThen(new WaitCommand(1))
                .andThen(setNormalGains())
                .andThen(setPivotPosition(Climber.DOWN));
    }

    public Command stow() {
        return setPivotServoPosition(0)
                .andThen(new WaitCommand(1))
                .andThen(setNormalGains())
                .andThen(setPivotPosition(Climber.STOW));
    }

    public Command climb() {
        return setPivotServoPosition(1)
                .andThen(new WaitCommand(1))
                .andThen(setClimbGains())
                .andThen(setPivotPosition(Climber.CLIMB));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
        Logger.recordOutput("Climber/goal", goal);

        io.setPivotPosition(goal);

        if (atSetpointTrigger().getAsBoolean()) {
            io.stop();
        }
    }
}
