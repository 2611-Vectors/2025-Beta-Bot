// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.Setpoints.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismSimulator;
import frc.robot.util.MechanismSimulatorActual;
import frc.robot.util.PhoenixUtil;
import frc.robot.util.TunablePIDController;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Arm extends SubsystemBase {
  private final TalonFX arm = new TalonFX(ARM_MOTOR_ID);

  DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(ARM_ENCODER_PORT);
  TunablePIDController armPID = new TunablePIDController(0.2, 0.0, 0.0, "/Testing/ArmPID/");

  public final ArmFeedforward armFF = new ArmFeedforward(0.0, 0.0, 0.0);
  LoggedNetworkNumber armKg;

  // This is temp
  LoggedNetworkNumber algaeProccesor_h,
      algaeP2_h,
      algaeP3_h,
      algaeProccesor_a,
      algaeP2_a,
      algaeP3_a,
      algaeSpeed;

  /** Creates a new Arm. */
  public Arm() {
    PhoenixUtil.configMotor(arm, false, NeutralModeValue.Brake);
    armPID.enableContinuousInput(-180, 180);
    // This is temp
    algaeProccesor_h =
        new LoggedNetworkNumber("SetpointTuning/Proccessor Height", PROCESSOR_HEIGHT);
    algaeProccesor_a = new LoggedNetworkNumber("SetpointTuning/Proccessor Angle", PROCCESOR_ANGLE);

    algaeP2_h = new LoggedNetworkNumber("SetpointTuning/Algae Pick 2 Height", ALGAE_PICK2_HEIGHT);
    algaeP2_a = new LoggedNetworkNumber("SetpointTuning/Algae Pick 2 Angle", ALGAE_PICK2_ANGLE);

    algaeP3_h = new LoggedNetworkNumber("SetpointTuning/Algae Pick 3 Height", ALGAE_PICK3_HEIGHT);
    algaeP3_a = new LoggedNetworkNumber("SetpointTuning/Algae Pick 3 Angle", ALGAE_PICK3_ANGLE);
    algaeSpeed = new LoggedNetworkNumber("SetpointTunning/AlgaeSpeed", ALGAE_INTAKE_SPEED);

    armKg = new LoggedNetworkNumber("Arm/Arm kG", 0.0);
  }

  /** Function for voltage control for arm motor */
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  public Command setArmVoltage(Supplier<Double> voltage) {
    return run(() -> arm.setVoltage(voltage.get()));
  }

  /**
   * Returns arm angle in units of degrees from -180 to 180 see
   * https://natedcoder.github.io/Arm-Simulation/ for visualiion
   */
  public double getPivotAngle() {
    // Using relative since the absolute encoder doesnt work
    // Clockwise should result in positive encoder values
    // double rotations = arm.getPosition().getValueAsDouble() / ARM_GEAR_RATIO;
    // double scaledRotation = rotations % 1;
    double scaledRotation = pivotEncoder.get();
    // Range is -180 to 180
    double degrees = (scaledRotation - 0.5) * 360 - PIVOT_ANGLE_OFFSET;
    if (degrees < -180) {
      degrees += 360;
    }
    if (degrees > 180) {
      degrees -= 360;
    }
    return degrees;
  }

  /** Function to calculate the relative distance between two angles */
  public static double getRelativeAngle(double angle1, double angle2) {
    double difference = (angle2 - angle1 + 180) % 360 - 180;
    Logger.recordOutput(
        "Angle/Arm Relative Angle", difference < -180 ? difference + 360 : difference);
    return difference < -180 ? difference + 360 : difference;
  }

  /** Function to flip the angle for other side of the robot */
  public static double flipAngle(double angle) {
    double reflectedAngle = -180 - angle;
    if (reflectedAngle < -180) {
      return reflectedAngle + 360;
    }
    return reflectedAngle;
  }

  /**
   * Sets the Arm Pivot to a target position and should be called periodically unit are in degrees
   */
  public Command setPivotAngle(Supplier<Double> angle) {
    return run(
        () -> {
          Logger.recordOutput("Arm/TargetAngle", angle.get());
          MechanismSimulator.updateArm(angle.get());

          double pidPart;
          pidPart = armPID.calculate(getPivotAngle(), angle.get());
          double ffPart =
              armFF.getKg() * Math.cos(Math.toRadians(getPivotAngle())); // armFF.getKg() *
          // Math.cos(Math.toRadians(getPivotAngle()));
          arm.setVoltage(MathUtil.clamp(pidPart + ffPart, -ARM_MAX_VOLTAGE, ARM_MAX_VOLTAGE));
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/current angle", getPivotAngle());
    Logger.recordOutput("Arm/Motor Position", arm.getPosition().getValueAsDouble());
    MechanismSimulatorActual.updateArm(getPivotAngle());
    armPID.update();

    // This is temp
    PROCESSOR_HEIGHT = algaeProccesor_h.get();
    PROCCESOR_ANGLE = algaeProccesor_a.get();

    ALGAE_PICK2_ANGLE = algaeP2_a.get();
    ALGAE_PICK2_HEIGHT = algaeP2_h.get();

    ALGAE_PICK3_ANGLE = algaeP3_a.get();
    ALGAE_PICK3_HEIGHT = algaeP3_h.get();

    ALGAE_INTAKE_SPEED = algaeSpeed.get();

    armFF.setKg(armKg.get());
  }
}
