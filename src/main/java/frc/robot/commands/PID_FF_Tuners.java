// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.Mechanisms.Elevator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class PID_FF_Tuners {
  public static Command ElevatorFFTuner(Elevator m_Elevator, Supplier<Double> joystick) {
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Elevator/Voltage", 0.0);
    return Commands.run(() -> m_Elevator.setVoltage(voltage.get() + joystick.get()), m_Elevator);
  }

  public static Command ArmFFTuner(Arm m_Arm, Supplier<Double> joystick) {
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("/Arm/Voltage", 0.0);
    return Commands.run(
        () -> {
          m_Arm.setArmVoltage(
              voltage.get() * Math.cos(Math.toRadians(m_Arm.getPivotAngle())) + joystick.get());
          Logger.recordOutput(
              "Arm FeedForward", voltage.get() * Math.cos(Math.toRadians(m_Arm.getPivotAngle())));
        },
        m_Arm);
  }

  public static Command ElevatorPIDTuning(Elevator m_Elevator) {
    LoggedNetworkNumber targetPosition = new LoggedNetworkNumber("/Elevator/TargetPosition", 0.0);
    return m_Elevator.setElevatorPosition(() -> targetPosition.get());
  }

  public static Command ArmPIDTuning(Arm m_Arm) {
    LoggedNetworkNumber targetAngle = new LoggedNetworkNumber("/Arm/TargetAngle", -99);
    return m_Arm.setPivotAngle(() -> targetAngle.get());
  }
}
