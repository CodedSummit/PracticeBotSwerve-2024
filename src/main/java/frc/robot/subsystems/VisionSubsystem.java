// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
 
  PhotonCamera m_frontCamera = new PhotonCamera(VisionConstants.kFrontCamName);
  
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
/**


    var result = m_camera.getLatestResult();
    // System.out.println("Camera results:"+ result.toString());
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      int targetNumber = target.getFiducialId();
      // System.out.println(" Target number:"+targetNumber+" Yaw, pitch:" + target.getYaw()+ "  " + target.getPitch());
    }
    getRange();
    **/
  }

 
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  /**
   * Get a target (if any) for the given ID
   * 
   */
  public PhotonTrackedTarget getTargetForTag(int AprilTagFiducialID){
    PhotonTrackedTarget target = null;
    var photonRes = m_frontCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
           
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == AprilTagFiducialID)
          .findFirst();
      if (targetOpt.isPresent()) {
        target = targetOpt.get();
      }
      
    }
    return target;
  }
}
