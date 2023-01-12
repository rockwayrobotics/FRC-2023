// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {
  PhotonCamera camera;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem() {
    camera = new PhotonCamera("photonvision");


  }

  @Override
  public void periodic() {
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if(hasTargets) {
      PhotonTrackedTarget target = result.getBestTarget();

      // Get information from target.
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      List<TargetCorner> corners = target.getDetectedCorners();

       // AprilTag specific data
      int targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

      System.out.print("Yaw: ");
      System.out.println(yaw);
      System.out.print("Pitch: ");
      System.out.println(pitch);
      System.out.print("Area: ");
      System.out.println(area);
      System.out.print("Skew: ");
      System.out.println(skew);
      System.out.print("Corners: ");
      System.out.println(corners);
      System.out.print("TargetID: ");
      System.out.println(targetID);
      System.out.print("Pose Ambiguity: ");
      System.out.println(poseAmbiguity);
      System.out.print("Best Camera to Target: ");
      System.out.println(bestCameraToTarget);
      System.out.print("Alternate Camera to Target: ");
      System.out.println(alternateCameraToTarget);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
