// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ScoringCommands;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.CustomAutoBuilder;
import frc.robot.util.TunablePIDController;

import static frc.robot.Constants.AutonConstants.*;

public class AlignReef extends SequentialCommandGroup {
    private static TunablePIDController drivePID_X = new TunablePIDController(0.7, 0, 0, "/tuning/driveX/");
    private static TunablePIDController drivePID_Y = new TunablePIDController(0.7, 0, 0, "tuning/driveY/");

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
    Pose2d closestPoint = getClosestPoint(m_Drive.getPose());
    Pose2d targetPos = CustomAutoBuilder.applyOffset(closestPoint, reefSide);

    addCommands(
      Commands.parallel(
            Commands.run(
                () -> {
                  drivePID_X.update();
                  drivePID_Y.update();
                  Logger.recordOutput("/targetPosition", targetPos);
                }),
            DriveCommands.joystickDriveAtAngle(
                m_Drive,
                () ->
                    MathUtil.clamp(
                        drivePID_X.calculate(m_Drive.getPose().getX(), targetPos.getX()), -.5, .5),
                () ->
                    MathUtil.clamp(
                        drivePID_Y.calculate(m_Drive.getPose().getY(), targetPos.getY()), -.5, .5),
                () -> closestPoint.getRotation()))
    );
  }
}
