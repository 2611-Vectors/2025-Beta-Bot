// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import frc.robot.subsystems.Mechanisms.EndEffector;

public class LoadStationIntake extends SequentialCommandGroup {
  public LoadStationIntake(
      RumbleConsumer consumer, Elevator m_Elevator, Arm m_Arm, EndEffector m_EndEffector) {
    super(
        m_Elevator.setUntil(() -> INTAKE_HEIGHT_IN),
        Commands.race(
            m_Elevator.setElevatorPosition(() -> INTAKE_HEIGHT_IN),
            m_Arm.setUntil(() -> INTAKE_ANGLE)),
        Commands.race(
            m_Arm.setPivotAngle(() -> INTAKE_ANGLE),
            m_Elevator.setElevatorPosition(() -> INTAKE_HEIGHT_IN),
            Commands.sequence(
                m_EndEffector.setEndEffectorVoltage(() -> -6.0),
                Commands.waitUntil(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) > 44),
                Commands.waitUntil(() -> Math.abs(m_EndEffector.getEndEffectorRPS()) < 31),
                Commands.runOnce(
                    () -> {
                      if (consumer != null) consumer.setRumble(RumbleType.kBothRumble, 1);
                    }),
                m_EndEffector.setEndEffectorVoltage(() -> 0.0))));
  }

  @FunctionalInterface
  public static interface RumbleConsumer {
    public void setRumble(RumbleType type, double value);
  }
}
