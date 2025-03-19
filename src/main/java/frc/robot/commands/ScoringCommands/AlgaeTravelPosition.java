// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.ALGAE_INTAKE_SPEED;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import static frc.robot.Constants.Setpoints.PROCCESOR_ANGLE;
import static frc.robot.Constants.Setpoints.PROCESSOR_HEIGHT;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

public class AlgaeTravelPosition extends SequentialCommandGroup {
  /** Creates a new AlgaeTravelPosition. */
  public AlgaeTravelPosition(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        m_EndEffector.setEndEffectorVoltage(() -> ALGAE_INTAKE_SPEED),
        m_Elevator.setUntil(() -> 40.0),
        Commands.race(
            m_Arm.setUntil(() -> PROCCESOR_ANGLE), m_Elevator.setElevatorPosition(() -> 40.0)),
        Commands.parallel(
            m_Arm.setPivotAngle(() -> PROCCESOR_ANGLE),
            m_Elevator.setElevatorPosition(() -> PROCESSOR_HEIGHT)));
  }
}
