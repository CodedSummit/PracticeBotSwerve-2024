package frc.robot.commands;

import edu.wpi.first.epilogue.Logged;
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

public class SpinTowardTarget extends Command {

  private final TurretSubsystem turretSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private double goalAngleDeg; // goal pose angle, degrees
  private static final double ANGLE_TOLERANCE = 1.0; // amount goal must change
  private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(.6, 8);
  private final ProfiledPIDController m_omegaController = new ProfiledPIDController(.5, 0, 0, OMEGA_CONSTRAINTS);

  public SpinTowardTarget(SwerveSubsystem swerveSubsystem, TurretSubsystem turretSubsystem) {

    this.turretSubsystem = turretSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    m_omegaController.setTolerance(Units.degreesToRadians(5));
    m_omegaController.enableContinuousInput(-1, 1);

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    var robotPose = swerveSubsystem.getPose();
    m_omegaController.reset(robotPose.getRotation().getRadians());
    checkGoal();
  }

  @Override
  public void execute() {
    spin();
  }

  /**
   * Turn the robot to the target pose rotation
   */
  private void spin() {
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
    if (m_omegaController.atGoal()) {
      // if we're at the goal we're done
      System.out.println("Reached the spin target goal - STOPPING");
      setSpeeds(0.0);
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // swerveSubsystem.stopModules();
    goalAngleDeg = 0.0;
  }

  private void checkGoal() {

    Pose2d robotPose = swerveSubsystem.getPose();
    Pose2d robotToTargetPose = turretSubsystem.getTargetPose(); //.relativeTo(robotPose);
    double rotInDeg = Math.atan2(robotToTargetPose.getY(), robotToTargetPose.getX()) * 180.0 / Math.PI; // goal pose
                                                                                                        // rotation
    rotInDeg *= -1;
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
    m_omegaController.setGoal(goalAngleDeg * Math.PI / 180.0);
  }
}