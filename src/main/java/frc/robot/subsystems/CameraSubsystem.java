// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.commands.DrivethAlign;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
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
  public static PhotonCamera camera;

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

    /**
   * Class that holds the directions that need to be followed to complete a shot.
   */
  public static class Direction {
    double x;
    public double y;
    double angle;
    public boolean ok;

    public Direction(double x1, double y1, double angle1, boolean ok1) {
      x = x1;
      y = y1;
      angle = angle1;
      ok = ok1;
    }

    @Override
    public String toString(){
        return "X: " + x + " Y: " + y + "\nAngle: " + angle + " OK: " + ok;
    }

  }

  private static PhotonTrackedTarget get_desired_target(List<PhotonTrackedTarget> target_list, int id){
    for (PhotonTrackedTarget target : target_list ){
      // System.out.println("ID: " + target.getFiducialId());
      if (target.getFiducialId() == id){
        return target;
      }
    }
    return null;
  }

  /**
   * Using the camera and April Tag finds the directions needed to complete a shot.
   * If no April Tags are found, the Direction.ok will be False.
   * @return Direction
   */
  public static Direction get_directions(){
    //Eventaully will be given dynamically
    int id = 0;
    PhotonPipelineResult result = camera.getLatestResult();
    double angle_thresh = 0.1; //in radians
    double x = 0;
    double y = 0;
    double angle = 0;
    boolean ok = result.hasTargets();
    if (ok){
      PhotonTrackedTarget target = get_desired_target(result.getTargets(), id);
      if (target != null){
        Transform3d three_d = target.getBestCameraToTarget();
        x = three_d.getX();
        y = three_d.getY();
        angle = Math.atan(y/x);
        System.out.println("xyz: " + x + " " + y + " " + Units.radiansToDegrees(angle));
      }

    }
    if (angle < angle_thresh){
      angle = 0;
    }
    return new Direction(x, y, angle, ok);
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
