// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import static frc.robot.Constants.ElevatorConstants.STARTING_HEIGHT;
import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoringCommands.LoadStationIntake;
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;
import frc.robot.util.CustomAutoBuilder;

public class Test3Piece extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public Test3Piece(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, Climb m_Climb) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();
    addCommands(
        Commands.parallel(
            Commands.sequence(
                Commands.race(m_Climb.runWinch(() -> 0.15), Commands.waitSeconds(3)),
                m_Climb.runWinch(() -> 0d)),
            Commands.sequence(
                Commands.race(
                    drivePaths[0],
                    m_Arm.setPivotAngle(() -> TRAVEL_ANGLE),
                    m_Elevator.setSmartElevatorPosition(() -> L4_HEIGHT_IN)),
                AutoScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE),
                scorePiece(m_Elevator, m_Arm, m_EndEffector, drivePaths[1], drivePaths[2]),
                scorePiece(m_Elevator, m_Arm, m_EndEffector, drivePaths[3], drivePaths[4]),
                new TravelPosition(m_Elevator, m_Arm, m_EndEffector))));
  }

  private Command scorePiece(
      Elevator m_Elevator,
      Arm m_Arm,
      EndEffector m_EndEffector,
      Command drivePath1,
      Command drivePath2) {
    return Commands.sequence(
        Commands.parallel(
            drivePath1,
            Commands.sequence(
                m_Elevator
                    .setSmartElevatorPosition(() -> INTAKE_HEIGHT_IN)
                    .until(
                        () ->
                            Math.abs(INTAKE_HEIGHT_IN - m_Elevator.getLeftElevatorPosition())
                                < POSITION_TOLERANCE),
                new LoadStationIntake(m_Elevator, m_Arm, m_EndEffector))),
        Commands.race(
            drivePath2,
            Commands.sequence(
                new WaitCommand(0.5),
                m_Arm.setPivotAngle(() -> TRAVEL_ANGLE),
                m_Elevator.setSmartElevatorPosition(() -> L4_HEIGHT_IN))),
        AutoScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE));
  }

  private Command AutoScoreSetpoint(
      Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, double height, double angle) {
    return Commands.sequence(
        m_Elevator
            .setSmartElevatorPosition(() -> height)
            .until(
                () -> Math.abs(height - m_Elevator.getLeftElevatorPosition()) < POSITION_TOLERANCE),
        Commands.parallel(
                m_Elevator.setSmartElevatorPosition(() -> height), m_Arm.setPivotAngle(() -> angle * 2))
            .until(() -> Math.abs(Arm.getRelativeAngle(angle * 2, m_Arm.getPivotAngle())) < 10),
        Commands.parallel(
                m_Elevator.setSmartElevatorPosition(() -> height), m_Arm.setPivotAngle(() -> angle))
            .until(
                () ->
                    Math.abs(Arm.getRelativeAngle(angle, m_Arm.getPivotAngle())) < ANGLE_TOLERANCE),
        m_EndEffector.setEndEffectorVoltage(() -> 6.0),
        Commands.race(
            m_Elevator.setSmartElevatorPosition(() -> height),
            m_Arm.setPivotAngle(() -> angle),
            Commands.waitSeconds(0.25)),
        Commands.race(
                m_Elevator.setSmartElevatorPosition(() -> height),
                m_Arm.setPivotAngle(() -> TRAVEL_ANGLE))
            .until(
                () ->
                    Math.abs(Arm.getRelativeAngle(TRAVEL_ANGLE, m_Arm.getPivotAngle()))
                        < ANGLE_TOLERANCE),
        m_EndEffector.setEndEffectorVoltage(() -> 0.0));
  }
}
