package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetWheelAlignment;
import frc.robot.commands.ZeroOdometry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.FrontLeft.DriveMotor,
            DriveConstants.FrontLeft.TurningMotor,
            DriveConstants.FrontLeft.DriveEncoderReversed,
            DriveConstants.FrontLeft.TurningEncoderReversed,
            DriveConstants.FrontLeft.TurningAbsoluteEncoder,
            DriveConstants.FrontLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.FrontRight.DriveMotor,
            DriveConstants.FrontRight.TurningMotor,
            DriveConstants.FrontRight.DriveEncoderReversed,
            DriveConstants.FrontRight.TurningEncoderReversed,
            DriveConstants.FrontRight.TurningAbsoluteEncoder,
            DriveConstants.FrontRight.DriveAbsoluteEncoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.BackLeft.DriveMotor,
            DriveConstants.BackLeft.TurningMotor,
            DriveConstants.BackLeft.DriveEncoderReversed,
            DriveConstants.BackLeft.TurningEncoderReversed,
            DriveConstants.BackLeft.TurningAbsoluteEncoder,
            DriveConstants.BackLeft.DriveAbsoluteEncoderReversed);

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.BackRight.DriveMotor,
            DriveConstants.BackRight.TurningMotor,
            DriveConstants.BackRight.DriveEncoderReversed,
            DriveConstants.BackRight.TurningEncoderReversed,
            DriveConstants.BackRight.TurningAbsoluteEncoder,
            DriveConstants.BackRight.DriveAbsoluteEncoderReversed);

    private final ADIS16470_IMU gyro = new ADIS16470_IMU();

    private final SwerveDrivePoseEstimator odometer = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getModulePositions(), new Pose2d());

    private VisionPoseEstimationSubsystem m_VisionPoseEstimationSubsystem;
    private double yTiltOffset;

    private MedianFilter yFilter = new MedianFilter(10);
    private double filtered_y;

    private GenericEntry turboSpeedFactor;
    private GenericEntry normalSpeedFactor;
    private GenericEntry dampenedSpeedFactor;

    private final Field2d m_field = new Field2d();

    public SwerveSubsystem() {
        this.initialize();
    }

    public SwerveSubsystem( VisionPoseEstimationSubsystem vPoseEstimation){
        m_VisionPoseEstimationSubsystem = vPoseEstimation;
        this.initialize();
    }

    public void initialize() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
        loadPreferences();

        ShuffleboardTab swerveTab = Shuffleboard.getTab("Swerve");
        swerveTab.add("Front Left", frontLeft)
            .withSize(2,2);
        swerveTab.add("Front Right", frontRight)
            .withSize(2,2);
        swerveTab.add("Back Right", backRight)
            .withSize(2,2)
            .withPosition(0, 2);
        swerveTab.add("Back Left", backLeft)
            .withSize(2,2)
            .withPosition(2, 2);
 
        swerveTab.add("Set Wheel Offsets", new SetWheelAlignment(this));
        
        turboSpeedFactor = swerveTab.add("Turbo Percentage", 0.9)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();

        normalSpeedFactor = swerveTab.add("Normal Percentage", .5)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();
 
        dampenedSpeedFactor = swerveTab.add("Dampened Percentage", .2)
            .withWidget(BuiltInWidgets.kNumberSlider) // specify the widget here
            .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
            .getEntry();

        swerveTab.add("Gyro data", gyro);

        swerveTab.add("Field", m_field);

        swerveTab.add(new ZeroOdometry(this));

// Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    0.1, //4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


    }

    public double getTurboSpeedFactor(){
        return turboSpeedFactor.getDouble(0.5);
    }
    public double getNormalSpeedFactor(){
        return normalSpeedFactor.getDouble(0.3);
    }
    public double getDampenedSpeedFactor(){
        return dampenedSpeedFactor.getDouble(0.1);
    }

    public void lockEncoderOffset(){
        frontLeft.lockEncoderOffset();
        frontRight.lockEncoderOffset();
        backLeft.lockEncoderOffset();
        backRight.lockEncoderOffset();
    }

    public void loadPreferences(){
        frontLeft.loadPreferences();
        frontRight.loadPreferences();
        backLeft.loadPreferences();
        backRight.loadPreferences();
    }

    public void zeroHeading() {
        gyro.reset();
    }
    public void setHeading(double setAngle){
        //pigeon.setYaw(setAngle);
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(gyro.getYawAxis()), 360); 
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getEstimatedPosition();
    }
    private void resetPose(Pose2d pose2d1) {
        resetOdometry(pose2d1);
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
        gyro.setGyroAngleZ(pose.getRotation().getDegrees());
    }
    public void resetOdometry(){
        resetOdometry(new Pose2d());
    }

    private SwerveModulePosition [] getModulePositions(){
        return new SwerveModulePosition[] {frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(),
            backRight.getPosition()};
    }

    @Override
    public void periodic() {
        
        updatePose();
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        m_field.setRobotPose(getPose());
        SmartDashboard.putNumber("PoseX", getPose().getX() );
        SmartDashboard.putNumber("PoseY", getPose().getY() );
    }

    public void updatePose(){
        m_VisionPoseEstimationSubsystem.updatePoseWithVision(odometer);
        odometer.update(getRotation2d(), getModulePositions());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState [] statelist = {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
        return statelist;
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub
        super.initSendable(builder);

    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());

    }

    public void driveRobotRelative(ChassisSpeeds chassisspeeds1) {
         // 5. Convert chassis speeds to individual module states
         SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisspeeds1);

         // 6. Output each module states to wheels
         setModuleStates(moduleStates);
    }

    public Command followPathCommand(String pathName) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        System.out.println("  Generating path for file:"+pathName);
        return new FollowPathHolonomic(
                path,
                this::getPose, // Robot pose supplier
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(2.0, 0.0, 0.0), // Rotation PID constants
                        0.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

}