package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/*
 *  rotated toward a fixed position on the field
 */
@Logged
public class SpinTowardTarget extends Command {

  @NotLogged
  private final TurretSubsystem turretSubsystem;
  @NotLogged
  private final SwerveSubsystem swerveSubsystem;

  private double goalAngleDeg; // goal pose angle, degrees
  private static final double ANGLE_TOLERANCE = 5.0; // amount goal must change
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(.6, 8);
  private  PIDController m_omegaController = new PIDController(.5, 0, 0);

  public SpinTowardTarget(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    m_omegaController.setTolerance(Units.degreesToRadians(5));
    m_omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    turretSubsystem.pointToTarget();
  }

  /**
   * Turn the robot to the target pose rotation
   */
  public boolean isFinished(){
    //return turretSubsystem.isFinished();
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    turretSubsystem.end(interrupted);
  }

}