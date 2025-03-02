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

import static frc.robot.Constants.AutonConstants.LEFT_OFFSET;
import static frc.robot.Constants.AutonConstants.RIGHT_OFFSET;
import static frc.robot.Constants.Setpoints.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autons.Left3Auton;
import frc.robot.commands.ScoringCommands.AlignReef;
import frc.robot.commands.ScoringCommands.LoadStationIntake;
import frc.robot.commands.ScoringCommands.ScoreSetpoint;
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
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
  private final Climb m_Climb;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController buttonBoard = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                    VisionConstants.BackRightCam, 
                    VisionConstants.robotToBackRightCam));
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

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    m_Drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_Drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    buttonBoard.leftBumper()
        .whileTrue(Commands.defer(() -> new AlignReef(m_Drive, LEFT_OFFSET), Set.of(m_Drive)));
    buttonBoard.rightBumper()
        .whileTrue(Commands.defer(() -> new AlignReef(m_Drive, RIGHT_OFFSET), Set.of(m_Drive)));

    buttonBoard.a()
        .whileTrue(new ScoreSetpoint(m_Elevator, m_Arm, L2_HEIGHT_IN, L2_ANGLE))
        .onFalse(new TravelPosition(m_Elevator, m_Arm));

    buttonBoard.x()
        .whileTrue(new ScoreSetpoint(m_Elevator, m_Arm, L2_HEIGHT_IN, L2_ANGLE))
        .onFalse(new TravelPosition(m_Elevator, m_Arm));

    buttonBoard.b()
        .whileTrue(new ScoreSetpoint(m_Elevator, m_Arm, L3_HEIGHT_IN, L3_ANGLE))
        .onFalse(new TravelPosition(m_Elevator, m_Arm));

    buttonBoard.y()
        .whileTrue(new ScoreSetpoint(m_Elevator, m_Arm, L4_HEIGHT_IN, L4_ANGLE))
        .onFalse(new TravelPosition(m_Elevator, m_Arm));

    buttonBoard.leftStick()
        .whileTrue(new LoadStationIntake(m_Elevator, m_Arm))
        .onFalse(new TravelPosition(m_Elevator, m_Arm));

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
    controller.x().onTrue(Commands.runOnce(m_Drive::stopWithX, m_Drive));

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
    return autoChooser.get();
    // return new Left3Auton(m_Elevator, m_Arm);
  }
}
