// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import frc.robot.commands.DrivethAlign;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Vision;
import frc.robot.Constants.Field;

public class CameraSubsystem extends SubsystemBase {
  public PhotonCamera camera;

  public PhotonPoseEstimator photonPoseEstimator;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    ArrayList<AprilTag> tagList = new ArrayList<AprilTag>();
    tagList.add(Field.tag0);
    tagList.add(Field.tag1);

    camera = new PhotonCamera(Vision.camName);

    AprilTagFieldLayout aprilTagFieldLayout = null;
    try {
      if(Field.realField) {
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
      } else {
        aprilTagFieldLayout = new AprilTagFieldLayout(tagList, Field.length, Field.width);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, Vision.robotToCam);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d lastEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(lastEstimatedRobotPose);

    return photonPoseEstimator.update();
  }

  @Override
  public void periodic() {
    // PhotonPipelineResult result = camera.getLatestResult();
    // if(result.hasTargets()) {
    //   SmartDashboard.putNumber("X", result.getBestTarget().getBestCameraToTarget().getX());
    //   SmartDashboard.putNumber("Y", result.getBestTarget().getBestCameraToTarget().getY());
      
    // }
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
