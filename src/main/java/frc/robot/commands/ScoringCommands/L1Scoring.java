// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.L2_ANGLE;
import static frc.robot.Constants.Setpoints.L2_HEIGHT_IN;
import static frc.robot.Constants.Setpoints.POSITION_TOLERANCE;
import static frc.robot.Constants.Setpoints.PROCCESOR_ANGLE;
import static frc.robot.Constants.Setpoints.TRAVEL_ANGLE;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

// Had to make a seperate scoring sequence just for L1/L2 because the string is in the way
public class L1Scoring extends SequentialCommandGroup {
  /** Creates a new L1Scoring. */
  public L1Scoring(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        Commands.race(
            m_Elevator
                .setElevatorPosition(() -> 40.0)
                .until(() -> Math.abs(40 - m_Elevator.getElevatorPosition()) < POSITION_TOLERANCE),
            m_Arm.setPivotAngle(() -> TRAVEL_ANGLE)),
        m_Arm
            .setPivotAngle(() -> L2_ANGLE)
            .until(
                () ->
                    Math.abs(Arm.getRelativeAngle(L2_ANGLE, m_Arm.getPivotAngle()))
                        < PROCCESOR_ANGLE),
        Commands.race(
            m_Elevator
                .setElevatorPosition(() -> L2_HEIGHT_IN)
                .until(
                    () ->
                        Math.abs(L2_HEIGHT_IN - m_Elevator.getElevatorPosition())
                            < POSITION_TOLERANCE),
            m_Arm.setPivotAngle(() -> L2_ANGLE)),
        Commands.race(m_Elevator.setElevatorPosition(() -> L2_HEIGHT_IN), new WaitCommand(0.3)),
        m_EndEffector.setEndEffectorVoltage(() -> 2.0),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> L2_HEIGHT_IN),
            m_Arm.setPivotAngle(() -> L2_ANGLE),
            Commands.waitSeconds(0.5)),
        Commands.race(
            m_Elevator
                .setElevatorPosition(() -> 40.0)
                .until(() -> Math.abs(40 - m_Elevator.getElevatorPosition()) < POSITION_TOLERANCE),
            m_Arm.setPivotAngle(() -> L2_ANGLE)),
        m_EndEffector.setEndEffectorVoltage(() -> 0.0),
        Commands.race(
            m_Arm
                .setPivotAngle(() -> TRAVEL_ANGLE)
                .until(
                    () ->
                        Math.abs(Arm.getRelativeAngle(TRAVEL_ANGLE, m_Arm.getPivotAngle()))
                            < PROCCESOR_ANGLE)));
  }
}
