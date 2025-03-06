// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import static frc.robot.Constants.AutonConstants.*;
import static frc.robot.Constants.VisionConstants.FIELD_HEIGHT;
import static frc.robot.Constants.VisionConstants.FIELD_WIDTH;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Mechanisms.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.TunablePIDController;
import org.littletonrobotics.junction.Logger;

public class AlignReef extends SequentialCommandGroup {
  private static TunablePIDController drivePID_X =
      new TunablePIDController(2.75, 0, 0.2, "/tuning/driveX/");
  private static TunablePIDController drivePID_Y =
      new TunablePIDController(2.75, 0, 0.2, "tuning/driveY/");

  private static Pose2d getClosestPoint(Pose2d curPose) {
    Pose2d closestPoint = new Pose2d();
    double closestDistance = Double.MAX_VALUE;
    for (Pose2d position : poseAngleMap.keySet()) {
      double dist = position.getTranslation().getDistance(curPose.getTranslation());
      if (dist < closestDistance) {
        closestDistance = dist;
        closestPoint = position;
      }
    }

    return closestPoint;
  }

  /** Creates a new AlignReef. */
  public AlignReef(Drive m_Drive, double reefSide) {
    Pose2d startPoint = m_Drive.getPose();
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      startPoint =
          (new Pose2d(
              FIELD_WIDTH - startPoint.getX(),
              FIELD_HEIGHT - startPoint.getY(),
              Rotation2d.fromDegrees(Arm.flipAngle(startPoint.getRotation().getDegrees()))));
    }
    System.out.println("--------");
    System.out.println(FIELD_WIDTH - startPoint.getX());
    System.out.println(FIELD_HEIGHT - startPoint.getY());
    System.out.println(m_Drive.getPose());
    System.out.println("--------");

    Pose2d closestPoint = getClosestPoint(startPoint);
    Pose2d targetPos = CustomAutoBuilder.applyOffset(closestPoint, reefSide);
    addCommands(
        Commands.parallel(
            Commands.run(
                () -> {
                  drivePID_X.update();
                  drivePID_Y.update();
                  Logger.recordOutput("/targetPosition", targetPos);
                }),
            AutoBuilder.followPath(
                CustomAutoBuilder.getPathFromPoints(startPoint, targetPos, 2.0))));
  }
}
