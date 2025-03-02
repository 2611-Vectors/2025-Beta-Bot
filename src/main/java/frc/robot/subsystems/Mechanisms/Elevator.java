// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.Elevator.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismSimulator;
import frc.robot.util.MechanismSimulatorActual;
import frc.robot.util.PhoenixUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final PIDController elevatorPID = new PIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
  public final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.0, 0.45, 0.0);

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(RIGHT_ELEVATOR_ID);

    PhoenixUtil.configMotor(leftMotor, true, NeutralModeValue.Brake);
    PhoenixUtil.configMotor(rightMotor, false, NeutralModeValue.Brake);
  }

  /** Function for voltage control */
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  /** Sets the Elevator to a target position and should be called periodically unit are in inches */
  public Command setElevatorPosition(Supplier<Double> target) {
    return run(
        () -> {
          double targetActual = target.get();
          Logger.recordOutput("Elevator/TargetPosition", targetActual);

          // Simulator update
          MechanismSimulator.updateElevator(targetActual + 20.33);
          if (!MechanismSimulator.isLegalTarget()) {
            double offset = LOWEST_HEIGHT - MechanismSimulator.targetArmHeight();
            targetActual += offset;
          }

          double pidPart = elevatorPID.calculate(getLeftElevatorPosition(), targetActual);
          double ffPart = elevatorFF.calculate(targetActual);

          setVoltage(
              MathUtil.clamp(pidPart + ffPart, -ELEVATOR_MAX_VOLTAGE * 0.5, ELEVATOR_MAX_VOLTAGE));
        });
  }

  /**
   * Function to get the Elevator position uses the left by default but graphs both the left and
   * right encoder values
   */
  public double getLeftElevatorPosition() {
    return leftMotor.getPosition().getValueAsDouble() * ROTATIONS_TO_INCHES + STARTING_HEIGHT;
  }

  public double getRightElevatorPosition() {
    return rightMotor.getPosition().getValueAsDouble() * ROTATIONS_TO_INCHES + STARTING_HEIGHT;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Telemetry for position might be useful for the simulator hint hint hint
    Logger.recordOutput("Elevator/LeftEncoder", getLeftElevatorPosition());
    Logger.recordOutput("Elevator/RightEncoder", getRightElevatorPosition());
    MechanismSimulatorActual.updateElevator(getLeftElevatorPosition() + 20.33);
  }
}
