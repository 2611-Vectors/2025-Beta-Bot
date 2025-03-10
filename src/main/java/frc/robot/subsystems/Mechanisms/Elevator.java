// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.Setpoints.HOME_HEIGHT_IN;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismSimulator;
import frc.robot.util.MechanismSimulatorActual;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Elevator extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final TunablePIDController elevatorPID =
      new TunablePIDController(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, "/ElevatorPID/");
  public final ElevatorFeedforward elevatorFF = new ElevatorFeedforward(0.0, 0.45, 0.0);

  LoggedNetworkNumber housingDiamter;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(RIGHT_ELEVATOR_ID);

    PhoenixUtil.configMotorElevator(leftMotor, true, NeutralModeValue.Brake, 70);
    PhoenixUtil.configMotorElevator(rightMotor, false, NeutralModeValue.Brake, 70);

    housingDiamter = new LoggedNetworkNumber("/Elevator/Housing Diameter", STRING_HOUSING_DIAMETER);
  }

  /** Function for voltage control */
  public void setVoltage(double voltage) {
    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);
  }

  /** Command for voltage control */
  public Command setVoltage(Supplier<Double> voltage) {
    return run(
        () -> {
          leftMotor.setVoltage(voltage.get());
          rightMotor.setVoltage(voltage.get());
        });
  }

  public Command holdElevator() {
    return run(
        () -> {
          leftMotor.setVoltage(elevatorFF.getKg());
          rightMotor.setVoltage(elevatorFF.getKg());
        });
  }

  /** Sets the Elevator to a target position and should be called periodically unit are in inches */
  public Command setElevatorPosition(Supplier<Double> target) {
    return run(
        () -> {
          double targetActual = target.get();
          if (targetActual < STARTING_HEIGHT - 3.38) {
            targetActual = HOME_HEIGHT_IN;
          }
          Logger.recordOutput("Elevator/TargetPosition", targetActual);

          // // Simulator update
          MechanismSimulator.updateElevator(targetActual);
          // if (!MechanismSimulator.isLegalTarget()) {
          //   double offset = LOWEST_HEIGHT - MechanismSimulator.targetArmHeight();
          //   targetActual += offset;
          // }

          double pidPart = elevatorPID.calculate(getLeftElevatorPosition(), targetActual);
          double ffPart = elevatorFF.calculate(targetActual);
          if (Math.abs(getLeftElevatorPosition() - targetActual) < 0.25) {
            pidPart = 0;
            ffPart = elevatorFF.getKg();
          }
          Logger.recordOutput(
              "Elevator/VoltageApplied",
              MathUtil.clamp(pidPart + ffPart, -1.8, ELEVATOR_MAX_VOLTAGE));
          setVoltage(MathUtil.clamp(pidPart + ffPart, -1.8, ELEVATOR_MAX_VOLTAGE));
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

  public Supplier<Boolean> elevatorWithinTolerance() {
    return () -> (Math.abs(30d - getLeftElevatorPosition()) < POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Elevator/LeftEncoder", getLeftElevatorPosition());
    Logger.recordOutput("Elevator/RightEncoder", getRightElevatorPosition());
    MechanismSimulatorActual.updateElevator(getLeftElevatorPosition());
    elevatorPID.update();
  }
}
