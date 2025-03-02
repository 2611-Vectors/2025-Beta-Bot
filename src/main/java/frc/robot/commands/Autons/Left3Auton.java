// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoringCommands.LoadStationIntake;
import frc.robot.commands.ScoringCommands.ScoreSetpoint;
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.util.CustomAutoBuilder;

import static frc.robot.Constants.Setpoints.*;

public class Left3Auton extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public Left3Auton(Elevator m_Elevator, Arm m_Arm) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();

    addCommands(
        drivePaths[0],
        new ScoreSetpoint(m_Elevator, m_Arm, L3_HEIGHT_IN, L3_ANGLE),
        Commands.parallel(
            drivePaths[1],
            m_Arm.setPivotAngle(() -> HOME_ANGLE),
            Commands.sequence(new WaitCommand(0.25), new LoadStationIntake(m_Elevator, m_Arm))),
        Commands.race(
            drivePaths[2],
            Commands.sequence(new WaitCommand(1), m_Arm.setPivotAngle(() -> HOME_ANGLE))),
        new ScoreSetpoint(m_Elevator, m_Arm, L3_HEIGHT_IN, L3_ANGLE),
        new TravelPosition(m_Elevator, m_Arm));
  }
}
