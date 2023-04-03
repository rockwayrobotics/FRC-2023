// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
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

  SuppliedValueWidget<Boolean> m_distanceWidget = Shuffleboard.getTab("Subsystems").addBoolean("TURN", this::getInDistance)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .withProperties(Map.of("colorWhenFalse", "red", "colorWhenTrue", "green"));

  SuppliedValueWidget<Double> m_xWidget = Shuffleboard.getTab("Subsystems").addDouble("X", this::getX);
  SuppliedValueWidget<Double> m_yWidget = Shuffleboard.getTab("Subsystems").addDouble("Y", this::getY);


  boolean getInDistance() {
    var vision = CameraSubsystem.camera.getLatestResult();
    if (!vision.hasTargets()) {
      return false;
    }
    return vision.getBestTarget().getBestCameraToTarget().getX() < 5;
  }

  double getX() {
    var vision = CameraSubsystem.camera.getLatestResult();
    if (!vision.hasTargets()) {
      return Double.NaN;
    }
    return vision.getBestTarget().getBestCameraToTarget().getX();
  }

  double getY() {
    var vision = CameraSubsystem.camera.getLatestResult();
    if (!vision.hasTargets()) {
      return Double.NaN;
    }
    return vision.getBestTarget().getBestCameraToTarget().getY();
  }

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

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera, Vision.robotToCam);
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

    public Direction(double x1, double y1, double angle1) {
      x = x1;
      y = y1;
      angle = angle1;
    }

    @Override
    public String toString(){
        return "X: " + x + " Y: " + y + "\nAngle: " + angle;
    }

  }

  public static PhotonTrackedTarget get_desired_target(List<PhotonTrackedTarget> target_list, int id){
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
  public static Direction get_directions(PhotonTrackedTarget target){
    Transform3d three_d = target.getBestCameraToTarget();
    double x = three_d.getX();
    double y = three_d.getY();
    double angle = Math.atan2(y, x);
    System.out.println("xyz: " + x + " " + y + " " + Units.radiansToDegrees(angle));

    return new Direction(x, y, angle);
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    if(result.hasTargets()) {
      SmartDashboard.putNumber("Pose Estimator X", result.getBestTarget().getBestCameraToTarget().getX());
      SmartDashboard.putNumber("Pose Estimator Y", result.getBestTarget().getBestCameraToTarget().getY());
      var myResult = photonPoseEstimator.update(result);
      // photonPoseEstimator.
      if (myResult.isPresent()) {
        // do something if there is a pose
        SmartDashboard.putNumber("New pose estimator X:", myResult.get().estimatedPose.getX());
        SmartDashboard.putNumber("New pose estimator Y:", myResult.get().estimatedPose.getY());
        SmartDashboard.putNumber("New pose estimator yaw:", Units.radiansToDegrees(myResult.get().estimatedPose.getRotation().getZ()));        
      } else {

      }
      // System.out.println("New pose estimator: " + myResult.);

    }

    // var myResult = photonPoseEstimator.update(result);

    // SmartDashboard.putNumber("New pose estimator X", myResult.get());

    // System.out.println("New pose estimator: " + myResult.get());

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
