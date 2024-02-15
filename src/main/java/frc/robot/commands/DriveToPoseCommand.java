package frc.robot.commands;


import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Command to drive to a given robot pose
 */

public class DriveToPoseCommand extends Command {
  
  private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.3, 32);
  private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.3, 32);
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRATINTS = 
      new TrapezoidProfile.Constraints(.3, 8);
  
  private final SwerveSubsystem m_drivetrainSubsystem;
  
  private final ProfiledPIDController m_xController = new ProfiledPIDController(4, 0, 0, X_CONSTRAINTS);
  private final ProfiledPIDController m_yController = new ProfiledPIDController(4, 0, 0, Y_CONSTRAINTS);
  private final ProfiledPIDController m_omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRATINTS);

  private Pose2d m_goalPose;
  private double m_xRobotPose;
  private double m_yRobotPose;
  private DoubleLogEntry m_Xlog;
  private DoubleLogEntry m_Ylog;
  private DoubleLogEntry m_Omegalog;

  

  public DriveToPoseCommand(
        Pose2d goalPose2d, 
        SwerveSubsystem drivetrainSubsystem) {
    
    this.m_drivetrainSubsystem = drivetrainSubsystem;
    this.m_goalPose = goalPose2d;
    m_xController.setTolerance(0.05);
    m_yController.setTolerance(0.05);
    m_omegaController.setTolerance(Units.degreesToRadians(5));
    m_omegaController.enableContinuousInput(-1, 1);

    addRequirements(drivetrainSubsystem);
    }

  @Override
  public void initialize() {

    var robotPose = m_drivetrainSubsystem.getPose();
    m_omegaController.reset(robotPose.getRotation().getRadians());
    m_xController.reset(robotPose.getX());
    m_yController.reset(robotPose.getY());
    setupShuffleboard();
    updateGoal(m_goalPose);
    DataLog log = DataLogManager.getLog();
    m_Xlog = new DoubleLogEntry(log, "XVel");
    m_Ylog = new DoubleLogEntry(log, "YVel");
    m_Omegalog = new DoubleLogEntry(log, "OmegaVel");
    

  }
  private void setupShuffleboard() {
    ShuffleboardTab vision = Shuffleboard.getTab("Drive");
    try {
      vision.add("m_xController", m_xController);
      vision.add("m_yController", m_yController);
      vision.add("omegaController", m_omegaController);
    } catch (Exception e) {// eat it.  for some reason it fails if the tab exists
    }
  }

  @Override
  public void execute() {
      driveToPose();
  }
/**
   *  Update the goal to the provided pose (in Robot centered coordinates)
   */
  public void updateGoal(Pose2d newGoal){
    System.out.println(" Updating goal in DriveCommand");
    m_goalPose = newGoal;
    if (m_goalPose != null) {
      // only reset the goal when there is a new goal
      m_xController.setGoal(m_goalPose.getX());
      m_yController.setGoal(m_goalPose.getY());
      m_omegaController.setGoal(m_goalPose.getRotation().getRadians());
    }
  }
  /**
   *  Drives the robot to the known pose (in Robot centered coordinates)
   */
  private void driveToPose(){
    var robotPose = m_drivetrainSubsystem.getPose();
   
    double xSpeed = m_xController.calculate(robotPose.getX());
    if (m_xController.atGoal()) {
      xSpeed = 0;
    }

    double ySpeed = m_yController.calculate(robotPose.getY());
    if (m_yController.atGoal()) {
      ySpeed = 0;
    }
    
    m_xRobotPose = robotPose.getX();
    m_yRobotPose = robotPose.getY();
    double omegaSpeed = m_omegaController.calculate(robotPose.getRotation().getRadians());
    if (m_omegaController.atGoal()) {
      omegaSpeed = 0;
    }
    
    m_Xlog.append(xSpeed);
    m_Ylog.append(ySpeed);
    m_Omegalog.append(omegaSpeed);
    ChassisSpeeds goalSpeeds = new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    SmartDashboard.putData("Robot pose", this);
    m_drivetrainSubsystem.driveRobotRelative(goalSpeeds);
  }
  
  public boolean isFinished(){

    if (m_xController.atGoal() && m_yController.atGoal() && m_omegaController.atGoal()) {
      // if we're at the goal in all dimensions, we're done
      System.out.println("Reached the Vision target goal - STOPPING");
      return true;
    }
    return false;
  }
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stopModules();
    m_goalPose = null;   // clear out the old goal in case command object is reused
  }

  public void initSendable(SendableBuilder builder) {
    // 
    builder.addDoubleProperty("Robot Pose X", () -> m_xRobotPose , (n) -> m_xRobotPose =n);
    builder.addDoubleProperty("Robot Pose Y", () -> m_yRobotPose , (n) -> m_yRobotPose = n);
  }


}

