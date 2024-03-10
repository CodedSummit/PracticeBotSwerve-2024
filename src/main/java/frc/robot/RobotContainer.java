// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AddressableLedSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionPoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final AddressableLedSubsystem m_led = new AddressableLedSubsystem(20,9);
  private VisionPoseEstimationSubsystem m_visionPoseEstimationSubsystem = new VisionPoseEstimationSubsystem(m_led);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(m_visionPoseEstimationSubsystem);

  private SwerveJoystickCmd swerveJoystickCmd;
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    swerveJoystickCmd = new SwerveJoystickCmd(
      swerveSubsystem,
      m_driverController);
    swerveSubsystem.setDefaultCommand(swerveJoystickCmd); 

    // make the chasetag command

    Command placeholderChaser = new ChaseTagCommand(m_visionSubsystem, swerveSubsystem, m_led);
    
    configureBindings();

    //   for debugging only - make a default chase command
   // CommandScheduler.getInstance().setDefaultCommand(m_visionSubsystem, placeholderChaser);

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("poseestimator", m_visionPoseEstimationSubsystem);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    m_driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));

    Command navToA = makeNavCommand(new Pose2d(1.81, 7.68, new Rotation2d(0)));
    m_driverController.a().whileTrue(navToA);

    m_driverController.x().whileTrue(new ChaseTagCommand(m_visionSubsystem, swerveSubsystem, m_led));

    // Left Bumper controls field orientation for drive mode. Upressed (default) is field oriented
    //     Pressed is robot oriented
    m_driverController.leftBumper()
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(false)))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setFieldOriented(true)));

    m_driverController.rightBumper()
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getDampenedSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    m_driverController.axisGreaterThan(3, 0.5)
      .onTrue(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getTurboSpeedFactor())))
      .onFalse(new InstantCommand(() -> swerveJoystickCmd.setMotionScale(swerveSubsystem.getNormalSpeedFactor())));

    m_driverController.povDown().onTrue(new InstantCommand(() ->m_led.setStripBlue()));
    m_driverController.povUp().onTrue(new InstantCommand(() ->m_led.setStripPurple()));

    // temporarily do while true so releasing button stops the path
    m_driverController.povLeft().whileTrue(swerveSubsystem.followPathCommand("ShortRun"));
  }

    public void runStartupCalibration(){
    /*if(!armSubsystem.isCalibrated()){
      new CalibrateArmCommand(armSubsystem).schedule();
    }*/
  }

  /**
   * @param targetPose
   * @return
   */
  public Command makeNavCommand(Pose2d targetPose){

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
            0.5, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

    // See the "Follow a single path" example for more info on what gets passed here
    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        constraints,
        0.0, // Goal end velocity in meters/sec
        0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
    );

    return pathfindingCommand;
  }

  public Command makePathCommand(String pathName){
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

      // Create a path following command using AutoBuilder. This will also trigger event markers.
      return AutoBuilder.followPath(path);
  };

  public void loadPreferences(){
    swerveSubsystem.loadPreferences();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
    //return new NothingCommand();
  }
}
