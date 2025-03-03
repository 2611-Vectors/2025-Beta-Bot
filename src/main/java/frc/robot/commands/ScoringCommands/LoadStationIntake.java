// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

public class LoadStationIntake extends SequentialCommandGroup {
  public LoadStationIntake(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        m_Elevator
            .setElevatorPosition(() -> INTAKE_HEIGHT_IN)
            .until(
                () ->
                    Math.abs(INTAKE_HEIGHT_IN - m_Elevator.getLeftElevatorPosition())
                        < POSITION_TOLERANCE),
        Commands.parallel(
                m_Elevator.setElevatorPosition(() -> INTAKE_HEIGHT_IN),
                m_Arm.setPivotAngle(() -> INTAKE_ANGLE))
            .until(
                () ->
                    Math.abs(Arm.getRelativeAngle(INTAKE_ANGLE, m_Arm.getPivotAngle()))
                        < ANGLE_TOLERANCE),
        Commands.parallel(
            m_Arm.setPivotAngle(() -> INTAKE_ANGLE),
            m_Elevator.setElevatorPosition(() -> INTAKE_HEIGHT_IN),
            Commands.sequence(
                m_EndEffector
                    .setEndEffectorVoltage(() -> -6.0)
                    .until(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) > 9),
                Commands.waitUntil(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) < 4),
                m_EndEffector.setEndEffectorVoltage(() -> 0.0))),
        Commands.parallel(m_Elevator.holdElevator(), m_Arm.setPivotAngle(() -> INTAKE_ANGLE)));
  }
}
