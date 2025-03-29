// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.TRAVEL_ANGLE;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TravelPositionFlipped extends SequentialCommandGroup {
  /** Creates a new TravelPositionFlipped. */
  public TravelPositionFlipped(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        m_Elevator.setUntil(() -> 39.0),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> 39.0), m_Arm.setUntil(() -> -TRAVEL_ANGLE)));
  }
}
