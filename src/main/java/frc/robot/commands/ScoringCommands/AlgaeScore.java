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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeScore extends SequentialCommandGroup {
  /** Creates a new AlgaeScore. */
  public AlgaeScore(
      Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector, double height, double angle) {
    addCommands(
        Commands.race(
                m_Arm.setArmVoltage(() -> 0d),
                m_Elevator
                    .setElevatorPosition(() -> 35.0)
                    .until(() -> m_Elevator.getElevatorPosition() > 30.0))
            .onlyIf(
                () -> Math.abs(Arm.getRelativeAngle(PROCCESOR_ANGLE, m_Arm.getPivotAngle())) > 10),
        Commands.race(m_Arm.setUntil(() -> PROCCESOR_ANGLE), m_Elevator.holdElevator()),
        Commands.race(
            m_Elevator.setUntil(() -> PROCESSOR_HEIGHT),
            m_Arm.setPivotAngle(() -> PROCCESOR_ANGLE)),
        Commands.parallel(
            new HoldPosition(m_Elevator, m_Arm, height, angle),
            m_EndEffector.setEndEffectorVoltage(() -> -ALGAE_INTAKE_SPEED)));
  }
}
