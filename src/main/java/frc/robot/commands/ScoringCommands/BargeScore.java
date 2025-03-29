// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.ALGAE_INTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BargeScore extends SequentialCommandGroup {
  /** Creates a new BargeScore. */
  public BargeScore(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
        m_EndEffector.setEndEffectorVoltage(() -> ALGAE_INTAKE_SPEED),
        new WaitCommand(0.5),
        Commands.waitUntil(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) < 5),
        Commands.runOnce(() -> m_Elevator.setCurrent(90)),
        Commands.race(
            m_Elevator
                .setElevatorPosition(() -> 69.0)
                .until(() -> m_Elevator.getElevatorPosition() > 67),
            m_Arm.setPivotAngle(() -> 110.0)),
        Commands.parallel(
            m_EndEffector.setEndEffectorVoltage(() -> -12.0),
            m_Elevator.setElevatorPosition(() -> 69.0),
            m_Arm.setPivotAngle(() -> 110.0)));
  }
}
