// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbCommand extends SequentialCommandGroup {
  /** Creates a new ClimbCommand. */
  public ClimbCommand(Elevator m_Elevator, Arm m_Arm, Climb m_Climb, EndEffector m_EndEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        m_EndEffector.setEndEffectorVoltage(() -> 0d),
        m_Elevator
            .setElevatorPosition(() -> 35d)
            .until(() -> Math.abs(35d - m_Elevator.getElevatorPosition()) < POSITION_TOLERANCE),
        Commands.race(
            m_Elevator.holdElevator(),
            m_Arm
                .setPivotAngle(() -> CLIMB_ANGLE)
                .until(
                    () ->
                        Math.abs(Arm.getRelativeAngle(CLIMB_ANGLE, m_Arm.getPivotAngle()))
                            < ANGLE_TOLERANCE)),
        m_Elevator
            .setElevatorPosition(() -> CLIMB_HEIGHT_IN)
            .until(
                () ->
                    Math.abs(CLIMB_HEIGHT_IN - m_Elevator.getElevatorPosition())
                        < POSITION_TOLERANCE),
        Commands.parallel(
            m_Elevator.setVoltage(() -> 0d),
            m_Climb.runGrab(() -> -1d),
            m_Arm.setPivotAngle(() -> CLIMB_ANGLE)));
  }
}
