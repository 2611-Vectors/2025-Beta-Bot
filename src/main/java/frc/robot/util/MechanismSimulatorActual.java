// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class MechanismSimulatorActual {
  static Mechanism2d elevatorMech = new Mechanism2d(3, 3);
  static MechanismRoot2d elevatorRoot = elevatorMech.getRoot("Elevator", 1.5, 0);
  static MechanismLigament2d m_elevator =
      elevatorRoot.append(new MechanismLigament2d("elevator", 1, 90));
  static MechanismLigament2d m_arm =
      m_elevator.append(
          new MechanismLigament2d("wrist", 0.4572, 90, 6, new Color8Bit(Color.kPurple)));

  static LoggedNetworkNumber elevatorPosition, wristAngle;

  public static void init() {
    elevatorPosition = new LoggedNetworkNumber("/Tuning/ElevatorSim/ElevatorActualPosition", 0.5);
    wristAngle = new LoggedNetworkNumber("/Tuning/ElevatorSim/WristActualAngle", 90);
    SmartDashboard.putData("Mech2Dactual", elevatorMech);
  }

  public static void updateArm(double angle) {
    m_arm.setAngle(angle + 90);
  }

  public static void updateElevator(double position) {
    m_elevator.setLength(position / 39.37);
  }

  public static double actualArmHeight() {
    return ARM_LENGTH * (-Math.sin(Math.toRadians(m_arm.getAngle()))) + m_elevator.getLength();
  }
}
