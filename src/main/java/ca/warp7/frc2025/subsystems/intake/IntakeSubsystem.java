package ca.warp7.frc2025.subsystems.intake;

import ca.warp7.frc2025.Constants;
import ca.warp7.frc2025.Robot;
import ca.warp7.frc2025.util.LoggedTunableNumber;
import ca.warp7.frc2025.util.pitchecks.PitCheckable;
import ca.warp7.frc2025.util.pitchecks.PitChecker;
import ca.warp7.frc2025.util.pitchecks.PitChecks;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase implements PitCheckable {
    private final double rollerGearRatio = 36 / 16;
    private final double rollerKt = rollerGearRatio / DCMotor.getKrakenX60Foc(1).KtNMPerAmp;

    // Roller io
    private final RollersIO rollersIO;
    private final RollersIOInputsAutoLogged rollersInputs = new RollersIOInputsAutoLogged();

    // Sensors
    private final ObjectDectionIO topSensorIO;
    private final ObjectDectionIO frontSensorIO;

    private final ObjectDectionIOInputsAutoLogged topSensorInputs = new ObjectDectionIOInputsAutoLogged();
    private final ObjectDectionIOInputsAutoLogged frontSensorInputs = new ObjectDectionIOInputsAutoLogged();

    private final Alert disconnectedMotor;

    private final double topDistanceToCoral = 95;
    private final double frontTopDistanceToCoral = 50;

    private final LoggedTunableNumber processerTorque = new LoggedTunableNumber("Intake/torque", 0.60);

    public IntakeSubsystem(RollersIO rollersIO, ObjectDectionIO topSensorIO, ObjectDectionIO frontSensorIO) {
        this.rollersIO = rollersIO;
        this.topSensorIO = topSensorIO;
        this.frontSensorIO = frontSensorIO;

        disconnectedMotor = new Alert("Intake motor disconnected", AlertType.kError);

        PitChecker.registerCheck(this);
    }

    @Override
    public void periodic() {
        rollersIO.updateInputs(rollersInputs);
        topSensorIO.updateInputs(topSensorInputs);
        frontSensorIO.updateInputs(frontSensorInputs);

        Logger.processInputs("Intake/Rollers", rollersInputs);
        Logger.processInputs("Intake/TopSensor", topSensorInputs);
        Logger.processInputs("Intake/FrontSensor", frontSensorInputs);

        disconnectedMotor.set(!rollersInputs.connected);
    }

    @AutoLogOutput
    public Trigger bottomSensorTrigger() {
        // return new Trigger(() -> MathUtil.isNear(topDistanceToCoral, topSensorInputs.objectDistanceMM, 50));
        return Constants.coral
                ? new Trigger(() -> MathUtil.isNear(topDistanceToCoral, topSensorInputs.objectDistanceMM, 50))
                : new Trigger(() -> topSensorInputs.objectDistanceMM <= 250);
    }

    @AutoLogOutput
    public Trigger middleSensorTrigger() {
        return Constants.coral
                ? new Trigger(() -> MathUtil.isNear(frontTopDistanceToCoral, frontSensorInputs.objectDistanceMM, 30))
                : new Trigger(() -> frontSensorInputs.objectDistanceMM <= 60);
    }

    @AutoLogOutput
    public Trigger combinedSensorTrigger() {
        return middleSensorTrigger().and(bottomSensorTrigger());
    }

    @AutoLogOutput
    public Command runVoltsRoller(double inputVolts) {
        return startEnd(() -> rollersIO.setVolts(inputVolts), () -> rollersIO.setVolts(0));
    }

    public Command setVoltsRoller(double volts) {
        return runOnce(() -> rollersIO.setVolts(volts));
    }

    @AutoLogOutput
    public Command runTorqueAmpsRoller(double inputAmps) {
        return runOnce(() -> rollersIO.setVolts(inputAmps));
    }

    private Command setBottomSensor(boolean value) {
        return runOnce(() -> {
            if (Robot.isSimulation()) topSensorInputs.objectDistanceMM = value ? topDistanceToCoral : 0;
        });
    }

    private Command setMiddleSensor(boolean value) {
        return runOnce(() -> {
            if (Robot.isSimulation()) frontSensorInputs.objectDistanceMM = value ? frontTopDistanceToCoral : 0;
        });
    }

    public Command intake() {
        return (Robot.isSimulation()
                        ? new WaitCommand(0.5)
                                .andThen(setMiddleSensor(true))
                                .andThen(setBottomSensor(true))
                                .andThen(new PrintCommand("Simulating intake"))
                        : Commands.none())
                .andThen(runVoltsRoller(-8).until(combinedSensorTrigger()));
    }

    public Command outake() {
        return outake(-12);
    }

    public Command outake(double volts) {
        return (Robot.isSimulation()
                        ? new WaitCommand(0.5)
                                .andThen(setMiddleSensor(false))
                                .andThen(setBottomSensor(false))
                                .andThen(new PrintCommand("Simulating outake"))
                        : Commands.none())
                .andThen(runVoltsRoller(volts)
                        .raceWith(new WaitCommand(0.3)
                                .andThen(new WaitUntilCommand(
                                        bottomSensorTrigger().negate()))));
    }

    public Trigger holdingCoral() {
        return middleSensorTrigger().and(bottomSensorTrigger());
    }

    public Trigger notHoldingCoral() {
        return middleSensorTrigger().negate().or(bottomSensorTrigger().negate());
    }

    public Command setTorque() {
        return setTorque(processerTorque.get());
    }

    public Command setTorque(double nM) {
        return run(() -> rollersIO.setTorqueAmps(nM * rollerKt));
    }

    public Command setTorque(DoubleSupplier nM) {
        return setTorque(nM.getAsDouble());
    }

    @Override
    public Command check() {
        return PitChecks.rampCheck(
                        0.75,
                        10.0,
                        0.1,
                        () -> rollersInputs.appliedVoltage,
                        (volts) -> rollersIO.setVolts(volts),
                        10,
                        "Intake")
                .andThen(PitChecks.goalCheck(
                        new double[] {1, 5, 10, -1, -5, -10},
                        0.1,
                        0,
                        (volts) -> setVoltsRoller(volts),
                        (volts) -> new Trigger(() -> rollersInputs.appliedVoltage == volts),
                        5,
                        "Intake"));
    }
}
