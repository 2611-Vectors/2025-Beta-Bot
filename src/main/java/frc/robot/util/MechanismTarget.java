// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.ArmConstants.*;
import static frc.robot.Constants.Setpoints.TRAVEL_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.Mechanisms.Arm;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

// COMBINE WITH MECHANISMsIMULATORaCTUAL

/** Add your docs here. */
public class MechanismTarget {
  /* Units in Meters */

  static Mechanism2d elevatorMech = new Mechanism2d(3, 3);
  static MechanismRoot2d elevatorRoot = elevatorMech.getRoot("Elevator", 1.5, 0);
  static MechanismLigament2d m_elevator =
      elevatorRoot.append(new MechanismLigament2d("elevator", 1, 90));
  static MechanismLigament2d m_arm =
      m_elevator.append(new MechanismLigament2d("arm", 0.6, 0, 6, new Color8Bit(Color.kPurple)));

  static LoggedNetworkNumber elevatorPosition, wristAngle;

  public static void init() {
    elevatorPosition = new LoggedNetworkNumber("/Tuning/ElevatorSim/ElevatorTargetPosition", 0.5);
    wristAngle = new LoggedNetworkNumber("/Tuning/ElevatorSim/WristTargetAngle", 90);
    SmartDashboard.putData("Mech2Dtarget", elevatorMech);
  }

  public static double armTargetAngle = TRAVEL_ANGLE;

  public static void updateArm(double angle) {
    armTargetAngle = angle;
    m_arm.setAngle(Arm.flipAngle(angle + 90));
  }

  public static void updateElevator(double position) {
    m_elevator.setLength(position / 39.37);
  }

  public static boolean isLegalTarget() {
    double armHeight =
        ARM_LENGTH * (-Math.sin(Math.toRadians(m_arm.getAngle()))) + m_elevator.getLength();
    return armHeight > 0.3;
  }

  public static double targetArmHeight() {
    return ARM_LENGTH * (-Math.sin(Math.toRadians(m_arm.getAngle()))) + m_elevator.getLength();
  }

  public static double targetArmAngle() {
    return armTargetAngle;
  }
}
