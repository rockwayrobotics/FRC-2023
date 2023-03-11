// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Map;

public class CameraSubsystem extends SubsystemBase {
  public static PhotonCamera camera;

  // TODO Rewrite to be more variable
  SuppliedValueWidget<Boolean> m_distanceWidget = Shuffleboard.getTab("Dashboard").addBoolean("TURN", this::getInDistance)
          .withWidget(BuiltInWidgets.kBooleanBox)
          .withProperties(Map.of("colorWhenFalse", "red", "colorWhenTrue", "green"));

  boolean getInDistance() {
//    System.out.println("Updating");

    var vision = CameraSubsystem.camera.getLatestResult();
    if (!vision.hasTargets()) {
      return false;
    }
    return vision.getBestTarget().getBestCameraToTarget().getX() < 5;
  }

  public CameraSubsystem() {
    camera = new PhotonCamera(Constants.Vision.camName);
  }

  public List<PhotonTrackedTarget> getTargetList() {
    PhotonPipelineResult cameraResult = camera.getLatestResult();
    if(cameraResult.hasTargets()) {
      return cameraResult.getTargets();
    } else {
      return null;
    }
  }

  public static PhotonTrackedTarget get_desired_target(List<PhotonTrackedTarget> target_list, int aprilTagID){
    for (PhotonTrackedTarget target : target_list ){
      if (target.getFiducialId() == aprilTagID){
        return target;
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
