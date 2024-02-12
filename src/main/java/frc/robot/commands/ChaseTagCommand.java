package frc.robot.commands;


import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ChaseTagCommand extends Command {
   
  // TODO:  Make a constructor that takes the TAG number, the TAG_TO_GOAL offsets, and the CAMERA_TO_ROBOT offsets
  
  private static final int TAG_TO_CHASE = 1;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0), Rotation2d.fromDegrees(180.0));
   /**
     * Physical location of the camera on the robot, relative to the camera.
     */
  public static final Transform2d CAMERA_TO_ROBOT = 
        new Transform2d(new Translation2d(inchesToMeters(-12.75), 0.0), new Rotation2d(0.0));

  private static final double STALE_TARGET_TIME = 2.0;   // how long to wait (sec.) after losing target before giving up
  private static final double DIFF_MAX_THRESHOLD = 0.50;   // Translation difference - in meters
  private static final double DIFF_MIN_RANGE = 0.2;   // Closest we'd get to the target (for scaling threshold)
  private static final double DIFF_MAX_RANGE = 3.0;   // Beyond this range (meters) use the max diff threshold
  private static final double OMEGA_DIFF_THRESHOLD = 5.0;  // Rotation difference - in degrees

  private final VisionSubsystem m_VisionSubsystem;
  private final SwerveSubsystem m_drivetrainSubsystem;
  

  private Pose2d m_goalPose;
  private PhotonTrackedTarget m_lastTarget;
  private double m_xRobotPose;
  private double m_yRobotPose;
  private Timer m_TargetLastSeen = null;

  private DriveToPoseCommand m_driveToPoseCmd = null;

  public ChaseTagCommand(
        VisionSubsystem visionSubsystem, 
        SwerveSubsystem drivetrainSubsystem) {
    this.m_VisionSubsystem = visionSubsystem;
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_driveToPoseCmd = new DriveToPoseCommand(null, drivetrainSubsystem);

    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    m_goalPose = null;
    m_lastTarget = null;
    
    m_driveToPoseCmd.updateGoal(null);
    setupShuffleboard();
    m_TargetLastSeen = new Timer();
  }
  private void setupShuffleboard() {
    ShuffleboardTab vision = Shuffleboard.getTab("Vision");
  }

  @Override
  public void execute() {
      var robotPose = m_drivetrainSubsystem.getPose();
      var target = m_VisionSubsystem.getTargetForTag(TAG_TO_CHASE);

      if (target == null && m_lastTarget == null){
        return; //  No target yet, and have not seen one yet.  Come back later.....
      }

      if (target != null){
        //We're still seeing a target
         // restart our timer on fresh target data (start() method is no-op if alredy running)
        m_TargetLastSeen.start();
        m_TargetLastSeen.reset();
      }
      
      if (target != null && targetDataSignificantlyDifferent(target, m_lastTarget)) {
        // This is new target data, so recalculate the goal
        m_lastTarget = target;
        System.out.println(" Calculating a new goal");
        // Get the transformation from the camera to the tag (in 2d)
        var camToTarget = target.getBestCameraToTarget();
        var transform = new Transform2d(
            camToTarget.getTranslation().toTranslation2d(),
            camToTarget.getRotation().toRotation2d());

        // Transform the robot's pose to find the tag's pose
        var cameraPose = robotPose.transformBy(CAMERA_TO_ROBOT.inverse());
        Pose2d targetPose = cameraPose.transformBy(transform);

        // Transform the tag's pose to set our goal
        m_goalPose = targetPose.transformBy(TAG_TO_GOAL);
        m_driveToPoseCmd.updateGoal(m_goalPose);
        if (!m_driveToPoseCmd.isScheduled()) m_driveToPoseCmd.schedule();
      }
  }

  /**
   *  Returns true if the newTarget is "significantly" different from lastTarget
   */
  private boolean targetDataSignificantlyDifferent(PhotonTrackedTarget newTarget, PhotonTrackedTarget lastTarget) {
    if (newTarget == null || lastTarget == null) return true;
    if (newTarget.equals(lastTarget)) return false;   //  exactly alike

    // see if the distances are different enough to care
    if (Math.abs(newTarget.getBestCameraToTarget().getX() - lastTarget.getBestCameraToTarget().getX()) > getDiffThreshold(newTarget)) return true;
    if (Math.abs(newTarget.getBestCameraToTarget().getY() - lastTarget.getBestCameraToTarget().getY()) > getDiffThreshold(newTarget)) return true;
    if (Units.radiansToDegrees(Math.abs(newTarget.getBestCameraToTarget().getRotation().getAngle() - 
          lastTarget.getBestCameraToTarget().getRotation().getAngle())) > OMEGA_DIFF_THRESHOLD) return true;
    
    return false;
  }

  /**
   * Calculate a scaled difference threshold.
   * As you get closer, the threshold decreases (allowing more frequent goal updates)
   */
  private double getDiffThreshold(PhotonTrackedTarget tgt){
    double x = tgt.getBestCameraToTarget().getX();
    double y = tgt.getBestCameraToTarget().getY();
    double range = Math.sqrt(x*x + y*y);
    if (range > DIFF_MAX_RANGE) return DIFF_MAX_THRESHOLD;    
    double thresh = (Math.max(range-DIFF_MIN_RANGE,0.01)/DIFF_MAX_RANGE) * DIFF_MAX_THRESHOLD;
  //  System.out.println(">>>>>>>Range, Threshold: "+range+"  "+thresh);
    return thresh;
  }

public boolean isFinished(){

   // if (m_driveToPoseCmd.isFinished()) return true;  // we got to where we're going

    if (m_TargetLastSeen.hasElapsed(STALE_TARGET_TIME)){
      // If we haven't gotten new target data in a while we may have lost sight of it and should stop
      System.out.println("Have not seen the target lately - STOPPING");
      return true;
    }
    return false;
  }
  @Override
  public void end(boolean interrupted) {
    m_driveToPoseCmd.cancel();
    m_TargetLastSeen.stop();
  }

  public void initSendable(SendableBuilder builder) {
    // 
    builder.addDoubleProperty("Robot Pose X", () -> m_xRobotPose , (n) -> m_xRobotPose =n);
    builder.addDoubleProperty("Robot Pose Y", () -> m_yRobotPose , (n) -> m_yRobotPose = n);
  }


}

