// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.ArmConstants.END_EFFECTOR_ID;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PhoenixUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private final TalonFX endEffector = new TalonFX(END_EFFECTOR_ID);
  // private final LinearFilter rpsFilter = LinearFilter.movingAverage(5);

  /** Creates a new EndEffector. */
  public EndEffector() {
    PhoenixUtil.configMotorElevator(endEffector, false, NeutralModeValue.Coast, 70);
  }

  public void setCurrent(double current) {
    PhoenixUtil.configMotorElevator(endEffector, false, NeutralModeValue.Coast, current);
  }

  public void setVoltage(double voltage) {
    endEffector.setVoltage(voltage);
  }

  public Command setEndEffectorVoltage(Supplier<Double> voltage) {
    return runOnce(() -> endEffector.setVoltage(voltage.get()));
  }

  public double getEndEffectorRPS() {
    // Logger.recordOutput(
    //     "Arm/Flitered End Effector RPS",
    //     rpsFilter.calculate(endEffector.getVelocity().getValueAsDouble()));
    return endEffector.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/End Effector RPS", getEndEffectorRPS());
  }
}
