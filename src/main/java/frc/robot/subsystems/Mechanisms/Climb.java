// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Mechanisms;

import static frc.robot.Constants.ClimbConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private final SparkMax winchMotor;

  private final SparkMax grabMotor;
  private final DigitalInput limitSwitch;

  public Climb() {
    winchMotor = new SparkMax(CLIMB_WINCH_ID, MotorType.kBrushless);
    grabMotor = new SparkMax(CLIMB_GRAB_ID, MotorType.kBrushless);

    limitSwitch = new DigitalInput(1);
  }

  public void setWinchVoltage(double voltage) {
    winchMotor.setVoltage(voltage);
  }

  public void setGrabVoltage(double voltage) {
    grabMotor.setVoltage(voltage);
  }

  public Boolean getLimitSwitch() {
    return !limitSwitch.get();
  }

  public Command runWinch(Supplier<Double> power) {
    return run(
        () -> {
          if (Math.abs(power.get()) > 0.1) {
            if (!limitSwitch.get()) {
              setWinchVoltage(0);
            } else {
              setWinchVoltage(power.get() * 12);
            }

          } else {
            setWinchVoltage(0);
          }
        });
  }

  public Command runWinchAuton(Supplier<Double> power) {
    return runOnce(
        () -> {
          setWinchVoltage(power.get());
        });
  }

  public Command runGrab(Supplier<Double> power) {
    return Commands.runOnce(() -> setGrabVoltage(power.get() * 8));
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Climb/Limit Switch", getLimitSwitch());
  }
}
