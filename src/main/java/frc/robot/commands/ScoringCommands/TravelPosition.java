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

public class TravelPosition extends SequentialCommandGroup {
  public TravelPosition(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        // This is a safety check to not move the arm until it above a certain hight
        Commands.race(
            m_Arm.setArmVoltage(() -> 0d),
            Commands.waitUntil(() -> m_Elevator.getElevatorPosition() > 37.0),
            m_Elevator.setElevatorPosition(() -> 38.0)),
        Commands.race(
            m_Arm
                .setPivotAngle(() -> TRAVEL_ANGLE)
                .until(
                    () ->
                        Math.abs(Arm.getRelativeAngle(TRAVEL_ANGLE, m_Arm.getPivotAngle()))
                            < ANGLE_TOLERANCE),
            m_Elevator.setElevatorPosition(() -> 38.0)),
        Commands.parallel(
                m_Arm.setPivotAngle(() -> TRAVEL_ANGLE),
                m_Elevator.setElevatorPosition(() -> TRAVEL_HEIGHT_IN))
            .until(
                () ->
                    Math.abs(TRAVEL_HEIGHT_IN - m_Elevator.getElevatorPosition())
                        < POSITION_TOLERANCE),
        Commands.parallel(
            m_Elevator.holdElevator(),
            m_Arm.setPivotAngle(() -> TRAVEL_ANGLE),
            m_EndEffector.setEndEffectorVoltage(() -> 0d)));
  }
}
