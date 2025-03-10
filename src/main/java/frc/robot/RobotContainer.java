// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.Constants.AutonConstants.*;
import static frc.robot.Constants.Setpoints.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Autons.Left3Auton;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PID_FF_Tuners;
import frc.robot.commands.ScoringCommands.AlgaeIntake;
import frc.robot.commands.ScoringCommands.AlgaeScore;
import frc.robot.commands.ScoringCommands.AlgaeTravelPosition;
import frc.robot.commands.ScoringCommands.AlignReef;
import frc.robot.commands.ScoringCommands.HoldPosition;
import frc.robot.commands.ScoringCommands.L1Scoring;
import frc.robot.commands.ScoringCommands.LoadStationIntake;
import frc.robot.commands.ScoringCommands.ScoreSetpoint;
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.commands.ScoringCommands.setElevatorHeight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.CustomAutoBuilder;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_Drive;
  private final Vision m_Vision;
  private final Elevator m_Elevator;
  private final Arm m_Arm;
  private final EndEffector m_EndEffector;
  private final Climb m_Climb;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController buttonBoard = new CommandXboxController(1);

  private final CommandXboxController programmingTestController = new CommandXboxController(2);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private Boolean manualMode = false;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        m_Drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                m_Drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.BackRightCam, VisionConstants.robotToBackRightCam),
                new VisionIOPhotonVision(
                    VisionConstants.FrontLeftCam, VisionConstants.robotToFrontLeftCam));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_Vision =
            new Vision(
                m_Drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.BackRightCam,
                    VisionConstants.robotToBackRightCam,
                    m_Drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        m_Drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_Vision = new Vision(m_Drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    m_Elevator = new Elevator();
    m_Arm = new Arm();
    m_EndEffector = new EndEffector();
    m_Climb = new Climb();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_Drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_Drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_Drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_Drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    autoChooser.addOption(
        "Camera Pos Calculator",
        Commands.sequence(
            Commands.runOnce(
                () -> m_Drive.setPose(new Pose2d(5.05, 5.24, Rotation2d.fromDegrees(60)))),
            Commands.parallel(
                Commands.run(
                    () ->
                        m_Vision.calculateCameraPositions(
                            () -> new Pose2d(5.05, 5.24, Rotation2d.fromDegrees(60)))))));
    autoChooser.addOption(
        "Tag Position Calculator",
        Commands.sequence(
            Commands.runOnce(
                () -> m_Drive.setPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)))),
            Commands.parallel(
                DriveCommands.joystickDrive(
                    m_Drive, () -> 0.0, () -> 0.0, () -> 0.0), // Spins slowly
                Commands.run(() -> m_Vision.calculateTagPositions(() -> m_Drive.getPose())))));

    // Configure the button bindings
    configureButtonBindings();
    configProgrammingButtonConfigs();
  }

  private void configProgrammingButtonConfigs() {
    programmingTestController
        .a()
        .onTrue(
            Commands.defer(
                () ->
                    Commands.sequence(
                        Commands.runOnce(() -> m_Vision.setPose(m_Drive.getPose())),
                        Commands.run(
                            () ->
                                m_Vision.calculateCameraPositions(
                                    () -> new Pose2d(5.05, 5.24, Rotation2d.fromDegrees(60))))),
                Set.of(m_Drive, m_Vision)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private final SlewRateLimiter slewRateX = new SlewRateLimiter(1.1);

  private final SlewRateLimiter slewRateY = new SlewRateLimiter(1.1);

  private void configureTestBindings() {
    m_Arm.setDefaultCommand(PID_FF_Tuners.ArmPIDTuning(m_Arm));
    // m_Arm.setDefaultCommand(PID_FF_Tuners.ArmFFTuner(m_Arm, () -> buttonBoard.getLeftY()));
    m_Elevator.setDefaultCommand(PID_FF_Tuners.ElevatorPIDTuning(m_Elevator));
    // m_EndEffector.setDefaultCommand(
    //     m_EndEffector.setEndEffectorVoltage(() -> buttonBoard.getLeftY() * (0.9 * 12)));
    // m_Elevator.setDefaultCommand(
    //     PID_FF_Tuners.ElevatorFFTuner(m_Elevator, () -> buttonBoard.getLeftY()));
    buttonBoard.start().whileTrue(m_Climb.runGrab(() -> -1.0)).onFalse(m_Climb.runGrab(() -> 0.0));
    m_Climb.setDefaultCommand(m_Climb.runWinch(() -> buttonBoard.getRightY()));
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_Drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_Drive,
            () -> -slewRateX.calculate(controller.getLeftY()),
            () -> -slewRateY.calculate(controller.getLeftX()),
            () -> -controller.getRightX()));

    // CLIMB CONTROLS
    // new Trigger(() -> buttonBoard.getRightY() > -0.6)
    //     .whileTrue(m_Climb.runGrab(() -> 8.0))
    //     .onFalse(m_Climb.runGrab(() -> 0d));
    buttonBoard
        .start()
        .onTrue(new ClimbCommand(m_Elevator, m_Arm, m_Climb, m_EndEffector))
        .onFalse(m_Climb.runGrab(() -> 0.0));
    // new Trigger(() -> buttonBoard.getRightY() < 0.6)
    //     .whileTrue(m_Climb.runWinch(() -> 0.5))
    //     .onFalse(m_Climb.runWinch(() -> 0d));

    buttonBoard
        .leftTrigger(0.8)
        .and(() -> controller.getHID().getLeftBumperButton())
        .whileTrue(Commands.defer(() -> new AlignReef(m_Drive, LEFT_OFFSET), Set.of(m_Drive)));
    buttonBoard
        .rightTrigger(0.8)
        .and(() -> controller.getHID().getLeftBumperButton())
        .whileTrue(Commands.defer(() -> new AlignReef(m_Drive, RIGHT_OFFSET), Set.of(m_Drive)));

    buttonBoard
        .leftBumper()
        .whileTrue(Commands.run(() -> m_EndEffector.setVoltage(6)))
        .onFalse(Commands.runOnce(() -> m_EndEffector.setVoltage(0)));

    buttonBoard
        .rightBumper()
        .whileTrue(Commands.run(() -> m_EndEffector.setVoltage(-6)))
        .onFalse(Commands.runOnce(() -> m_EndEffector.setVoltage(0)));

    buttonBoard
        .x()
        .whileTrue(
            Commands.defer(
                () ->
                    Commands.either(
                        new AlgaeIntake(
                            m_Elevator,
                            m_Arm,
                            m_EndEffector,
                            ALGAE_PICK2_HEIGHT,
                            ALGAE_PICK2_ANGLE),
                        new L1Scoring(m_Elevator, m_Arm, m_EndEffector)
                            .andThen(
                                new HoldPosition(
                                    m_Elevator, m_Arm, m_EndEffector, 40.0, TRAVEL_ANGLE, 0)),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)))
        .onFalse(
            Commands.defer(
                () ->
                    Commands.either(
                        new AlgaeTravelPosition(m_Elevator, m_Arm, m_EndEffector),
                        new TravelPosition(m_Elevator, m_Arm, m_EndEffector),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)));

    buttonBoard
        .a()
        .whileTrue(
            Commands.defer(
                () ->
                    Commands.either(
                        new AlgaeScore(
                            m_Elevator, m_Arm, m_EndEffector, PROCESSOR_HEIGHT, PROCCESOR_ANGLE),
                        new L1Scoring(m_Elevator, m_Arm, m_EndEffector)
                            .andThen(
                                new HoldPosition(
                                    m_Elevator, m_Arm, m_EndEffector, 40.0, TRAVEL_ANGLE, 0)),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)))
        .onFalse(
            Commands.defer(
                () ->
                    Commands.either(
                        new setElevatorHeight(m_Elevator, m_Arm, () -> 40d)
                            .andThen(new TravelPosition(m_Elevator, m_Arm, m_EndEffector))
                            .onlyIf(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) > 2),
                        new TravelPosition(m_Elevator, m_Arm, m_EndEffector),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)));

    buttonBoard
        .b()
        .whileTrue(
            Commands.defer(
                () ->
                    Commands.either(
                        new AlgaeIntake(
                            m_Elevator,
                            m_Arm,
                            m_EndEffector,
                            ALGAE_PICK3_HEIGHT,
                            ALGAE_PICK3_ANGLE),
                        new ScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L3_HEIGHT_IN, L3_ANGLE)
                            .andThen(
                                new HoldPosition(
                                    m_Elevator,
                                    m_Arm,
                                    m_EndEffector,
                                    L3_HEIGHT_IN,
                                    TRAVEL_ANGLE,
                                    0)),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)))
        .onFalse(
            Commands.defer(
                () ->
                    Commands.either(
                        new AlgaeTravelPosition(m_Elevator, m_Arm, m_EndEffector),
                        new TravelPosition(m_Elevator, m_Arm, m_EndEffector),
                        () -> buttonBoard.getHID().getBackButton()),
                Set.of(m_Elevator, m_Arm, m_EndEffector)));

    buttonBoard
        .y()
        .whileTrue(
            new ScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE)
                .andThen(
                    new HoldPosition(
                        m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, TRAVEL_ANGLE, 0)))
        .onFalse(new TravelPosition(m_Elevator, m_Arm, m_EndEffector));

    new Trigger(() -> buttonBoard.getLeftY() > 0.6)
        .and(() -> !manualMode)
        .whileTrue(
            new LoadStationIntake(m_Elevator, m_Arm, m_EndEffector)
                .andThen(
                    new HoldPosition(
                        m_Elevator, m_Arm, m_EndEffector, INTAKE_HEIGHT_IN, INTAKE_ANGLE, 0)))
        .onFalse(new TravelPosition(m_Elevator, m_Arm, m_EndEffector));

    SmartDashboard.putBoolean("Manual Mode", manualMode);
    buttonBoard
        .rightStick()
        .onTrue(Commands.runOnce(() -> manualMode = !manualMode).ignoringDisable(true))
        .onChange(
            Commands.run(() -> SmartDashboard.putBoolean("Manual Mode", manualMode))
                .ignoringDisable(true));

    controller
        .rightTrigger(0.8)
        .onTrue(Commands.runOnce(() -> manualMode = !manualMode).ignoringDisable(true))
        .onChange(
            Commands.run(() -> SmartDashboard.putBoolean("Manual Mode", manualMode))
                .ignoringDisable(true));

    // new Trigger(() -> manualMode)
    //     .whileTrue(
    //         Commands.parallel(
    //             m_Arm.setArmVoltage(() -> buttonBoard.getRightY()),
    //             m_Elevator.setVoltage(
    //                 () -> -buttonBoard.getLeftY() + m_Elevator.elevatorFF.getKg())))
    //     .whileFalse
    m_Climb.setDefaultCommand(m_Climb.runWinch(() -> buttonBoard.getRightY()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    controller.rightBumper().onTrue(Commands.runOnce(m_Drive::stopWithX, m_Drive));

    // Reset gyro to 0° when Back button is pressed
    controller
        .back()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_Drive.setPose(
                            new Pose2d(m_Drive.getPose().getTranslation(), new Rotation2d())),
                    m_Drive)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_Drive.setPose(CustomAutoBuilder.getStartPose2d());
    // return autoChooser.get();
    return new Left3Auton(m_Elevator, m_Arm, m_EndEffector, m_Climb);
    // return new Test3Piece(m_Elevator, m_Arm, m_EndEffector, m_Climb);
  }

  public void zeroMotors() {
    m_Elevator.setVoltage(0);
    m_Arm.setArmVoltage(0);
    m_EndEffector.setVoltage(0);
    m_Climb.setWinchVoltage(0);
    m_Climb.setGrabVoltage(0);
  }
}
