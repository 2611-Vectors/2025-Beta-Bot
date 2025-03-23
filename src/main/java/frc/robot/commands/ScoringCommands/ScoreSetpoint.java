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

public class ScoreSetpoint extends SequentialCommandGroup {
  public ScoreSetpoint(
      Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, double height, double angle) {
    super(
        m_Elevator
            .setElevatorPosition(() -> height)
            .until(() -> (m_Elevator.getElevatorPosition() > 30d || height < 30)),
        Commands.race(m_Elevator.setUntil(() -> height), m_Arm.setPivotAngle(() -> TRAVEL_ANGLE)),
        Commands.race(m_Elevator.setElevatorPosition(() -> height), m_Arm.setUntil(() -> angle)),
        m_EndEffector.setEndEffectorVoltage(() -> 2.0),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> height),
            m_Arm.setPivotAngle(() -> angle),
            Commands.waitSeconds(0.25)),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> height), m_Arm.setUntil(() -> TRAVEL_ANGLE)),
        m_EndEffector.setEndEffectorVoltage(() -> 0.0));
  }
}
