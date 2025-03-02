// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Elevator;

/** Add your docs here. */
public class PID_FF_Tuners {
    public static Command ElevatorFFTuner(Elevator m_Elevator) {
        LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Elevator/Voltage", 0.0);
        return Commands.run(() -> m_Elevator.setVoltage(voltage.get()), m_Elevator);
    }
}
