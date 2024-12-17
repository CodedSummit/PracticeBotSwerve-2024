package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveRotateToPiece extends SwerveJoystickCmd {

    private final VisionSubsystem visionSubsystem;

    public DriveRotateToPiece(SwerveSubsystem swerveSubsystem, CommandXboxController m_driverController, VisionSubsystem visionSubsystem) {
      super(swerveSubsystem, m_driverController);  
      this.visionSubsystem = visionSubsystem;

        this.turningSpdFunction = () -> turningSpeed();
        
    
    }

    private double turningSpeed(){
      if(this.visionSubsystem.hasGamePieceTarget() == false){
        return 0.0;
      }
      
      double s = this.visionSubsystem.getGamePieceTargetYaw()/-35;
      
      return MathUtil.clamp(s, -1, 1);

    }
}