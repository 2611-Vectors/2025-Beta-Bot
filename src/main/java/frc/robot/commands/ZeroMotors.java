// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Climb;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroMotors extends InstantCommand {
  Elevator m_Elevator;
  Arm m_Arm;
  EndEffector m_EndEffector;
  Climb m_Climb;

  public ZeroMotors(Elevator elevator, Arm arm, EndEffector endEffector, Climb climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = elevator;
    m_Arm = arm;
    m_EndEffector = endEffector;
    m_Climb = climb;

    addRequirements(m_Elevator, m_Arm, m_EndEffector, m_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Elevator.setVoltage(0);
    m_Arm.setArmVoltage(0);
    m_EndEffector.setVoltage(0);
    m_Climb.setWinchVoltage(0);
    m_Climb.setGrabVoltage(0);
  }
}
