// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package ca.warp7.frc2025;

import ca.warp7.frc2025.Constants.Climber;
import ca.warp7.frc2025.Constants.Intake;
import ca.warp7.frc2025.FieldConstants.ReefLevel;
import ca.warp7.frc2025.commands.DriveCommands;
import ca.warp7.frc2025.generated.TunerConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionConstants;
import ca.warp7.frc2025.subsystems.Vision.VisionIO;
import ca.warp7.frc2025.subsystems.Vision.VisionIOLimelight;
import ca.warp7.frc2025.subsystems.Vision.VisionSubsystem;
import ca.warp7.frc2025.subsystems.climber.ClimberIO;
import ca.warp7.frc2025.subsystems.climber.ClimberIOTalonFX;
import ca.warp7.frc2025.subsystems.climber.ClimberSubsystem;
import ca.warp7.frc2025.subsystems.drive.DriveSubsystem;
import ca.warp7.frc2025.subsystems.drive.GyroIO;
import ca.warp7.frc2025.subsystems.drive.GyroIOPigeon2;
import ca.warp7.frc2025.subsystems.drive.ModuleIO;
import ca.warp7.frc2025.subsystems.drive.ModuleIOSim;
import ca.warp7.frc2025.subsystems.drive.ModuleIOTalonFX;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIO;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIOSim;
import ca.warp7.frc2025.subsystems.elevator.ElevatorIOTalonFX;
import ca.warp7.frc2025.subsystems.elevator.ElevatorSubsystem;
import ca.warp7.frc2025.subsystems.intake.IntakeSubsystem;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIO;
import ca.warp7.frc2025.subsystems.intake.ObjectDectionIOLaserCAN;
import ca.warp7.frc2025.subsystems.intake.RollersIO;
import ca.warp7.frc2025.subsystems.intake.RollersIOSim;
import ca.warp7.frc2025.subsystems.intake.RollersIOTalonFX;
import ca.warp7.frc2025.subsystems.leds.LEDSubsystem;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure;
import ca.warp7.frc2025.subsystems.superstructure.Superstructure.SuperState;
import ca.warp7.frc2025.util.FieldConstantsHelper;
import ca.warp7.frc2025.util.pitchecks.PitChecker;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    private final DriveSubsystem drive;
    private final IntakeSubsystem intake;
    private final ElevatorSubsystem elevator;
    private final ClimberSubsystem climber;

    @SuppressWarnings("unused")
    private final VisionSubsystem vision;

    private final LEDSubsystem leds;
    // orchestra motors
    // private final TalonFX orchestraMotorRollers = new TalonFX(2); // Intake rollers

    // Superstructure
    private final Superstructure superstructure;

    // Controller
    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    enum Side {
        Left,
        Right
    }

    public Side side = Side.Left;

    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                drive = new DriveSubsystem(
                        new GyroIOPigeon2(
                                TunerConstants.DrivetrainConstants.Pigeon2Id,
                                TunerConstants.DrivetrainConstants.CANBusName),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                intake = new IntakeSubsystem(
                        new RollersIOTalonFX(2, "rio"),
                        new ObjectDectionIOLaserCAN(Intake.TOP_LASER_CAN),
                        new ObjectDectionIOLaserCAN(Intake.FRONT_LASER_CAN));

                climber = new ClimberSubsystem(
                        new ClimberIOTalonFX(Climber.PIVOT_ID, Climber.INTAKE_ID, Climber.Servo_PWM));

                elevator = new ElevatorSubsystem(new ElevatorIOTalonFX(11, 12));

                vision = new VisionSubsystem(
                        drive::addVisionMeasurement,
                        new VisionIOLimelight(VisionConstants.camera0Name, () -> drive.getRotation(), false),
                        new VisionIOLimelight(VisionConstants.camera1Name, () -> drive.getRotation(), false));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new DriveSubsystem(
                        new GyroIO() {},
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));

                intake = new IntakeSubsystem(
                        new RollersIOSim(DCMotor.getKrakenX60(1), 1, 0.01),
                        new ObjectDectionIO() {},
                        new ObjectDectionIO() {});

                elevator = new ElevatorSubsystem(new ElevatorIOSim());

                climber = new ClimberSubsystem(new ClimberIO() {});

                vision = new VisionSubsystem(drive::addVisionMeasurement);
                // new VisionIOPhotonVisionSim(
                //         VisionConstants.camera0Name,
                //         VisionConstants.robotToCamera0,
                //         () -> drive.getPose(),
                //         false),
                // new VisionIOPhotonVisionSim(
                //         VisionConstants.camera1Name,
                //         VisionConstants.robotToCamera1,
                //         () -> drive.getPose(),
                //         false));
                break;
            case REPLAY:
            default:
                drive = new DriveSubsystem(
                        new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});

                intake = new IntakeSubsystem(new RollersIO() {}, new ObjectDectionIO() {}, new ObjectDectionIO() {});

                elevator = new ElevatorSubsystem(new ElevatorIO() {});

                climber = new ClimberSubsystem(new ClimberIO() {});

                vision = new VisionSubsystem(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        }

        leds = new LEDSubsystem(2);
        leds.setToDefault();

        superstructure = new Superstructure(
                elevator,
                intake,
                leds,
                driveController.y().or(driveController.x()),
                driveController.rightTrigger(),
                driveController.leftTrigger(),
                driveController.rightBumper(),
                driveController.leftBumper(),
                new Trigger(() -> false),
                () -> FieldConstants.Reef.algaeLevels.get(FieldConstantsHelper.getclosestFace(drive.getPose())));

        configureNamedCommands();

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Autos", AutoBuilder.buildAutoChooser());

        if (Constants.tuningMode) {
            autoChooser.addOption(
                    "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
            autoChooser.addOption("Drive Wheel FF Characterization", DriveCommands.feedforwardCharacterization(drive));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption(
                    "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            if (DriverStation.getMatchType() == MatchType.None) {
                autoChooser.addOption("Pit Checks Sequence", PitChecker.runChecksInSequence());
                autoChooser.addOption("Pit Checks Parallel", PitChecker.runChecksInParallel());
                autoChooser.addOption("Intake Checks", intake.check());
                autoChooser.addOption("Elevator Checks", elevator.check());
            }
        }
        configureBindings();
    }

    private void configureNamedCommands() {
        Command Stow =
                elevator.setGoal(Constants.Elevator.STOW).andThen(new WaitUntilCommand(elevator.atSetpointTrigger()));

        NamedCommands.registerCommand("Stow", Stow);
        NamedCommands.registerCommand("L4", elevator.setGoal(Constants.Elevator.L4));
        NamedCommands.registerCommand("Intake", intake.intake());
        NamedCommands.registerCommand("Outake", intake.outake());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {
        Command driveCommand = DriveCommands.joystickDrive(
                drive,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX());

        Command scoreLeft = Commands.runOnce(() -> side = Side.Left);
        Command scoreRight = Commands.runOnce(() -> side = Side.Right);

        operatorController.y().onTrue(superstructure.setLevel(ReefLevel.L4));
        operatorController.x().onTrue(superstructure.setLevel(ReefLevel.L2));
        operatorController.b().onTrue(superstructure.setLevel(ReefLevel.L3));
        operatorController.a().onTrue(superstructure.setLevel(ReefLevel.L1));

        operatorController.povLeft().onTrue(scoreLeft);
        operatorController.povRight().onTrue(scoreRight);
        operatorController.back().onTrue(superstructure.forceState(SuperState.IDLE));
        operatorController.back().onTrue(superstructure.forceState(SuperState.IDLE));

        drive.setDefaultCommand(driveCommand);

        driveController.povRight().whileTrue(intake.runVoltsRoller(-10));
        driveController.povLeft().whileTrue(intake.runVoltsRoller(8));

        driveController.start().onTrue(drive.zeroPose());

        driveController.povUp().onTrue(climber.climb());
        driveController.povDown().onTrue(climber.down());

        driveController.a().onTrue(superstructure.forceState(SuperState.IDLE));
        driveController.rightBumper().onTrue(superstructure.forceState(SuperState.READY_CORAL));

        driveController.y().onTrue(intake.intake());
        // orchestra (create and add all talon motors)
        // Orchestra canada = new Orchestra();
        // Orchestra jazz = new Orchestra();

        // canada.addInstrument(orchestraMotorRollers);
        // jazz.addInstrument(orchestraMotorRollers);
        // var statusCan = canada.loadMusic("canada.chrp");
        // var statusJazz = jazz.loadMusic("jazz.chrp");
        // System.out.println(statusJazz);
        // System.out.println(statusCan);
        // operatorController.rightBumper().onTrue(new InstantCommand(() -> {
        //     if (canada.isPlaying()) {
        //         System.out.println("Pausing O Canada");
        //         canada.pause();
        //     } else {
        //         System.out.println("Playing O Canada");
        //         canada.play();
        //     }
        // }));

        // operatorController.leftBumper().onTrue(new InstantCommand(() -> {
        //     if (jazz.isPlaying()) {
        //         System.out.println("Pausing Jazz");
        //         jazz.pause();
        //     } else {
        //         System.out.println("Playing Jazz");
        //         jazz.play();
        //     }
        // }));
    }

    private void configureTuningBindings() {}

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
