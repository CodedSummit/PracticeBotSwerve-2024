// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
 
  private DoubleEntry m_targetX;
  private DoubleEntry m_targetY;
  private Pose2d m_target = new Pose2d(0.0, 0.0, new Rotation2d(0.0));

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    init();
  }

  private void init() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("datatable");
    m_targetX = table.getDoubleTopic("targetX").getEntry(0.0);
    m_targetY = table.getDoubleTopic("targetY").getEntry(0.0);
    m_targetX.set(0);
    m_targetY.set(0);
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
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

 
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
}
