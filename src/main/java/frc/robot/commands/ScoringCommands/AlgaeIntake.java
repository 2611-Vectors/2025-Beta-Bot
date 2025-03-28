// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.ALGAE_INTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntake extends SequentialCommandGroup {
  /** Creates a new AlgaeIntake. */
  public AlgaeIntake(
      Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, double height, double angle) {
    super(
        m_Elevator.setUntil(() -> height),
        Commands.race(m_Elevator.setElevatorPosition(() -> height), m_Arm.setUntil(() -> angle)),
        Commands.parallel(
            m_Arm.setPivotAngle(() -> angle),
            m_Elevator.setElevatorPosition(() -> height),
            m_EndEffector.setEndEffectorVoltage(() -> ALGAE_INTAKE_SPEED)));
  }
}
