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
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomAutoBuilder;
import java.util.function.Supplier;

public class Center1Piece extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public Center1Piece(
      Drive m_Drive, Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, Climb m_Climb) {
    Supplier<Command> startPath = () -> CustomAutoBuilder.generateInitialPath(m_Drive.getPose());

    addCommands(
        Commands.parallel(
            Commands.sequence(
                Commands.race(m_Climb.runWinch(() -> 0.15), Commands.waitSeconds(3)),
                m_Climb.runWinch(() -> 0d)),
            Commands.sequence(
                Commands.runOnce(() -> elevatorSlewRate.reset(m_Elevator.getElevatorPosition())),
                Commands.race(
                    startPath.get(),
                    m_Arm.setPivotAngle(() -> TRAVEL_ANGLE),
                    m_Elevator.setSlowElevatorPosition(() -> TRAVEL_HEIGHT_IN)),
                AutoScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE),
                new TravelPosition(m_Elevator, m_Arm, m_EndEffector))));
  }

  SlewRateLimiter elevatorSlewRate = new SlewRateLimiter(20, -20, STARTING_HEIGHT);

  private Command AutoScoreSetpoint(
      Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, double height, double angle) {
    return Commands.sequence(
        m_Elevator
            .setElevatorPosition(() -> height)
            .until(() -> Math.abs(height - m_Elevator.getElevatorPosition()) < POSITION_TOLERANCE),
        Commands.parallel(
                m_Elevator.setElevatorPosition(() -> height), m_Arm.setPivotAngle(() -> angle))
            .until(
                () ->
                    Math.abs(Arm.getRelativeAngle(angle, m_Arm.getPivotAngle())) < ANGLE_TOLERANCE),
        m_EndEffector.setEndEffectorVoltage(() -> 11.0),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> height),
            m_Arm.setPivotAngle(() -> angle),
            new WaitCommand(0.3)) // ,
        // Commands.race(
        //         m_Elevator.setElevatorPosition(() -> height),
        //         m_Arm.setPivotAngle(() -> TRAVEL_ANGLE))
        //     .until(
        //         () ->
        //             Math.abs(Arm.getRelativeAngle(TRAVEL_ANGLE, m_Arm.getPivotAngle()))
        //                 < ANGLE_TOLERANCE),
        // m_EndEffector.setEndEffectorVoltage(() -> 0.0)
        );
  }
}
