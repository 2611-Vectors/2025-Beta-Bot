// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  public static void configMotor(
      TalonFX motor,
      boolean inverted,
      NeutralModeValue motorOutput,
      PIDController pidController,
      ArmFeedforward feedforward) {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    if (pidController != null) {
      configs.Slot0.kP = pidController.getP(); // An error of 1 rotation results in 2.4 V output
      configs.Slot0.kI = pidController.getI(); // No output for integrated error
      configs.Slot0.kD = pidController.getD(); // A velocity of 1 rps results in 0.1 V output
    }

    if (feedforward != null) {
      configs.Slot0.kS =
          feedforward.getKs(); // Baseline voltage required to overcome static forces like friction
      configs.Slot0.kG = feedforward.getKg(); // Voltage to overcome gravity
    }
    configs.MotorOutput.Inverted =
        inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

    // Peak output of 8 V
    configs.Voltage.PeakForwardVoltage = 8;
    configs.Voltage.PeakReverseVoltage = -8;

    configs.CurrentLimits.StatorCurrentLimit = 60;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;

    // Example on how you would do break mode / coast mode
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(configs, 0.25));
    motor.setPosition(0);
  }

  public static void configMotor(TalonFX motor, boolean inverted, NeutralModeValue motorOutput) {
    configMotor(motor, inverted, motorOutput, null, null);
  }
}
