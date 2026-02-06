// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class TurretSubsystem extends SubsystemBase {
 
  private DoubleEntry m_targetX;
  private DoubleEntry m_targetY;
  private Pose2d m_target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
@NotLogged
  private final SwerveSubsystem swerveSubsystem;

  private double goalAngleDeg; // goal pose angle, degrees
  private static final double ANGLE_TOLERANCE = 5.0; // amount goal must change
  private  PIDController m_omegaController = new PIDController(.5, 0, 0);

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem(SwerveSubsystem swerveSS) {
    swerveSubsystem = swerveSS;
    init();
  }

  private void init() {
    SmartDashboard.putData(m_omegaController);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    m_targetX = table.getDoubleTopic("targetX").getEntry(0.0);
    m_targetY = table.getDoubleTopic("targetY").getEntry(0.0);
    m_targetX.set(0);
    m_targetY.set(0);
    checkGoal();

  }
  /* 
   * Set the field-coordinates target location the turret could aim at
   */
  public void setTarget(double x, double y){
    Rotation2d rot = new Rotation2d(0.0);
    m_target = new Pose2d(x, y, rot);
  }

  public Pose2d getTargetPose() {
    Rotation2d rot = new Rotation2d(0.0);
    m_target = new Pose2d(m_targetX.getAsDouble(), m_targetY.getAsDouble(), rot);
    return m_target;
  }
  
  public void pointToTarget() {
    spin();
  }

  /**
   * Turn the robot to the target pose rotation
   */
  private void spin() {
    if (isFinished()) {
      end(false);
      return;
    }
    checkGoal();
    Pose2d robotPose = swerveSubsystem.getPose();
    double omegaSpeed = m_omegaController.calculate(robotPose.getRotation().getRadians());
    setSpeeds(omegaSpeed);
  }

  private void setSpeeds(double rotationSpeed) {
    ChassisSpeeds currentChassisSpeeds = swerveSubsystem.getRobotRelativeSpeeds();
    ChassisSpeeds goalSpeeds = new ChassisSpeeds(currentChassisSpeeds.vxMetersPerSecond,
        currentChassisSpeeds.vyMetersPerSecond, rotationSpeed);
    swerveSubsystem.driveRobotRelative(goalSpeeds);
  }
  public boolean isFinished() {
    if (m_omegaController.atSetpoint()) {
      // if we're at the goal we're done
      System.out.println("Reached the spin target goal - STOPPING");
      setSpeeds(0.0);
      return true;
    }
    return false;
  }

  public void end(boolean interrupted) {
    System.out.println("spin to target END");

    goalAngleDeg = 0;
    setSpeeds(0.0);
  }

  private void checkGoal() {

    Pose2d robotPose = swerveSubsystem.getPose();
    double dX=this.m_targetX.getAsDouble() - robotPose.getX();
    double dY=this.m_targetY.getAsDouble() - robotPose.getY();
    double rotInDeg = Math.atan2(dY, dX) * 180.0 / Math.PI; // goal pose
    if (Math.abs(rotInDeg - goalAngleDeg) > ANGLE_TOLERANCE) {
      // goal changed
      System.out.println("  Calculated goal pose rotation:" + rotInDeg);
      updateGoal(rotInDeg);
    }

  }

  /**
   * Update the goal to the provided angle (in Field centered coordinates)
   */
  private void updateGoal(double newGoal) {
    goalAngleDeg = newGoal;
    m_omegaController.setSetpoint(goalAngleDeg * Math.PI / 180.0);
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
