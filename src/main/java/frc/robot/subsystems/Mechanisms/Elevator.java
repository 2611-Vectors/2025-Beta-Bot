// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.Setpoints.HOME_HEIGHT_IN;
import static frc.robot.Constants.Setpoints.POSITION_TOLERANCE;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismActual;
import frc.robot.util.MechanismTarget;
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

  private final Encoder elevatorEncoder = new Encoder(1, 2);

  LoggedNetworkNumber housingDiamter;

  /** Creates a new Elevator. */
  public Elevator() {
    leftMotor = new TalonFX(LEFT_ELEVATOR_ID);
    rightMotor = new TalonFX(RIGHT_ELEVATOR_ID);

    PhoenixUtil.configMotorElevator(leftMotor, true, NeutralModeValue.Brake, 75);
    PhoenixUtil.configMotorElevator(rightMotor, false, NeutralModeValue.Brake, 75);

    housingDiamter = new LoggedNetworkNumber("/Elevator/Housing Diameter", STRING_HOUSING_DIAMETER);

    elevatorProfiledController.reset(getElevatorPosition());
  }

  public void setCurrent(double current) {
    PhoenixUtil.configMotorElevator(leftMotor, true, NeutralModeValue.Brake, current);
    PhoenixUtil.configMotorElevator(rightMotor, false, NeutralModeValue.Brake, current);
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
          setVoltage(voltage.get());
        });
  }

  public Command holdElevator() {
    return run(
        () -> {
          setVoltage(elevatorFF.getKg());
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
          MechanismTarget.updateElevator(targetActual);

          double pidPart = elevatorPID.calculate(getElevatorPosition(), targetActual);
          double ffPart = elevatorFF.calculate(targetActual);
          // if (Math.abs(getElevatorPosition() - targetActual) < 0.25) {
          //   pidPart = 0;
          //   ffPart = elevatorFF.getKg();
          // }
          Logger.recordOutput(
              "Elevator/VoltageApplied",
              MathUtil.clamp(pidPart + ffPart, -1.8, ELEVATOR_MAX_VOLTAGE));
          setVoltage(MathUtil.clamp(pidPart + ffPart, -1.8, ELEVATOR_MAX_VOLTAGE));
        });
  }

  public Command setSlowElevatorPosition(Supplier<Double> target) {
    return run(
        () -> {
          double targetActual = target.get();
          if (targetActual < STARTING_HEIGHT - 3.38) {
            targetActual = HOME_HEIGHT_IN;
          }
          Logger.recordOutput("Elevator/TargetPosition", targetActual);

          // // Simulator update
          MechanismTarget.updateElevator(targetActual);

          double pidPart = elevatorPID.calculate(getElevatorPosition(), targetActual);
          double ffPart = elevatorFF.calculate(targetActual);
          // if (Math.abs(getElevatorPosition() - targetActual) < 0.25) {
          //   pidPart = 0;
          //   ffPart = elevatorFF.getKg();
          // }
          Logger.recordOutput("Elevator/VoltageApplied", MathUtil.clamp(pidPart + ffPart, -1.8, 2));
          setVoltage(MathUtil.clamp(pidPart + ffPart, -1.8, 2));
        });
  }

  private final ProfiledPIDController elevatorProfiledController =
      new ProfiledPIDController(
          elevatorPID.getP(),
          elevatorPID.getI(),
          elevatorPID.getD(),
          new TrapezoidProfile.Constraints(100, 100));

  public Command setSmartElevatorPosition(Supplier<Double> target) {
    elevatorProfiledController.reset(getElevatorPosition());
    elevatorProfiledController.setGoal(target.get());
    return run(
        () -> {
          double targetActual = target.get();
          if (targetActual < STARTING_HEIGHT) {
            targetActual = HOME_HEIGHT_IN;
          }
          Logger.recordOutput("Elevator/TargetPosition", targetActual);

          // // Simulator update
          MechanismTarget.updateElevator(targetActual);
          // if (!MechanismSimulator.isLegalTarget()) {
          //   double offset = LOWEST_HEIGHT - MechanismSimulator.targetArmHeight();
          //   targetActual += offset;
          // }
          double pidPart = elevatorProfiledController.calculate(getElevatorPosition());
          double ffPart = elevatorFF.calculate(targetActual);
          if (Math.abs(getElevatorPosition() - targetActual) < 0.25) {
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

  public double getElevatorPosition() {
    return (elevatorEncoder.getDistance() * Math.PI * STRING_HOUSING_DIAMETER * 1.416807) / 2048
        + STARTING_HEIGHT;
  }

  public Supplier<Boolean> elevatorWithinTolerance() {
    return () -> (Math.abs(30d - getElevatorPosition()) < POSITION_TOLERANCE);
  }

  public Command setUntil(Supplier<Double> target) {
    return setElevatorPosition(target)
        .until(() -> Math.abs(target.get() - getElevatorPosition()) < POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Logger.recordOutput("Elevator/LeftEncoder", getLeftElevatorPosition());
    // Logger.recordOutput("Elevator/RightEncoder", getRightElevatorPosition());

    // Logger.recordOutput(
    //     "Elevator/Left Elevator Supplied Current",
    // leftMotor.getSupplyCurrent().getValueAsDouble());
    // Logger.recordOutput(
    //     "Elevator/Right Elevator Supplied Current",
    //     rightMotor.getSupplyCurrent().getValueAsDouble());
    Logger.recordOutput("Elevator/Current Position", getElevatorPosition());
    MechanismActual.updateElevator(getElevatorPosition());
    elevatorPID.update();
  }
}
