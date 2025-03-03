// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autons;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoringCommands.LoadStationIntake;
import frc.robot.commands.ScoringCommands.ScoreSetpoint;
import frc.robot.commands.ScoringCommands.TravelPosition;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;
import frc.robot.util.CustomAutoBuilder;

public class Left3Auton extends SequentialCommandGroup {

  /** Creates a new L4Auton. */
  public Left3Auton(Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    Command[] drivePaths = CustomAutoBuilder.getDrivePaths();

    addCommands(
        drivePaths[0],
        new ScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE),
        Commands.parallel(
            drivePaths[1],
            Commands.sequence(
                new WaitCommand(0.25), new LoadStationIntake(m_Elevator, m_Arm, m_EndEffector))),
        Commands.race(
            drivePaths[2],
            Commands.sequence(new WaitCommand(1), m_Arm.setPivotAngle(() -> TRAVEL_ANGLE))),
        new ScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE),
        Commands.parallel(
            drivePaths[3],
            Commands.sequence(
                new WaitCommand(0.25), new LoadStationIntake(m_Elevator, m_Arm, m_EndEffector))),
        Commands.race(
            drivePaths[4],
            Commands.sequence(new WaitCommand(1), m_Arm.setPivotAngle(() -> TRAVEL_ANGLE))),
        new ScoreSetpoint(m_Elevator, m_Arm, m_EndEffector, L4_HEIGHT_IN, L4_ANGLE),
        new TravelPosition(m_Elevator, m_Arm, m_EndEffector));
  }
}
