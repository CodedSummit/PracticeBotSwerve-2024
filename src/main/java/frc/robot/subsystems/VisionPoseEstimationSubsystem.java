// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
/**
 *  Uses Vision data to come up with estimated poses that can be fed to the drive system pose estimator.
 *  Don't really need this to be a subsystem since it's doesn't need mutex protection - but keeping it as one fits 
 *  the design pattern
 */
public class VisionPoseEstimationSubsystem extends SubsystemBase {
 
  PhotonCamera m_frontCamera = new PhotonCamera(VisionConstants.kFrontCamName);
  AprilTagFieldLayout m_CompetitionAprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  PhotonPoseEstimator m_photonPoseEstimator=null;
  private boolean m_visionEnabled = false;

  /** Creates a new VisionPoseEstimationSubsystem. */
  public VisionPoseEstimationSubsystem() {
    // Construct PhotonPoseEstimator
     m_photonPoseEstimator = new PhotonPoseEstimator(m_CompetitionAprilTagFieldLayout, 
      PoseStrategy.AVERAGE_BEST_TARGETS, m_frontCamera, VisionConstants.kRobotToFrontCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void initialize() {
     ShuffleboardTab vTab = Shuffleboard.getTab("Vision");
     vTab.add("Enable vision ", m_visionEnabled)
     .withSize(1,1);

  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
      m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return m_photonPoseEstimator.update();
  }

  /**
   * Adds vision esimtates to the provided pose esimator
   * 
   * @param poseEstimator
   */

  public void updatePoseWithVision(SwerveDrivePoseEstimator poseEstimator) {

    if (getVisionEnable()) {
      // TODO: Update to get vision info from each camera and add to the poseEstimator
      var pose = getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (pose.isPresent()) {
        var pose2d = pose.get().estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(pose2d, pose.get().timestampSeconds);
        System.out.println(" Updated pose with vision.  x:" + pose2d.getX() + "   y: " + pose2d.getY());
      }
    }

  }

  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("Vision Location Enabled", this::getVisionEnable, this::enableVisionPose);
  }

  public boolean getVisionEnable(){
    return m_visionEnabled;
  }
  public boolean enableVisionPose(boolean b) {
    m_visionEnabled = b;
    return m_visionEnabled;
  }

}
