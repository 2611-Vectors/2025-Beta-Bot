// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.Setpoints.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import java.util.function.Supplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setElevatorHeight extends SequentialCommandGroup {
  /** Creates a new setElevatorHeight. */
  public setElevatorHeight(Elevator m_Elevator, Arm m_Arm, Supplier<Double> height) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        Commands.race(
            m_Arm.setArmVoltage(() -> 0d),
            m_Elevator
                .setElevatorPosition(() -> height.get())
                .until(
                    () ->
                        Math.abs(height.get() - m_Elevator.getLeftElevatorPosition())
                            < POSITION_TOLERANCE)));
  }
}
