package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
/*
 * Drive while staying rotated toward a fixed position on the field
 */
@Logged
public class DriveRotateToPosition extends SwerveJoystickCmd {

    private final TurretSubsystem turretSubsystem;
    private final SwerveSubsystem swerveSubsystem;

    public DriveRotateToPosition(SwerveSubsystem swerveSubsystem, CommandXboxController m_driverController, TurretSubsystem turretSubsystem) {
      super(swerveSubsystem, m_driverController);  
      this.turretSubsystem = turretSubsystem;
      this.swerveSubsystem = swerveSubsystem;

        this.turningSpdFunction = () -> turningSpeed();
        
    
    }

    private double turningSpeed(){
            
      Pose2d robotPose = swerveSubsystem.getPose();
      Pose2d robotToTargetPose = turretSubsystem.getTargetPose().relativeTo(robotPose);
      double rotInRad = Math.atan2(robotToTargetPose.getY(), robotToTargetPose.getX());
      rotInRad = rotInRad - robotPose.getRotation().getRadians();  // take out the existing robot rotation
      System.out.println("  Calculated rotation to target:"+rotInRad*180.0/Math.PI);
      double angle = (rotInRad*180.0/Math.PI)/35;

      
      return MathUtil.clamp(angle, -1, 1);
    }
}