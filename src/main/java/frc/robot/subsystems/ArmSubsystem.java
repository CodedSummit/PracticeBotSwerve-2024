// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

/**
 * Models the Arm that is used for putting Notes in the Amp, as well as
 * positioning the Climbing hook at the end of the match.
 * 
 * Also has an absolute position sensor and motor to change positions.
 * This angle change position will need a PID controller. Uses FeedForward in
 * addition to PID control
 * This will likely have some preset positions, but also need to use controller
 * buttons to feed forward/backward slowly to adjust (see Bump functions).
 * 
 * Must be able to feed a Note either direction - forward into an Amp, Backward into the Stage Trap at the end of match
 * 
 * see example at
 * https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbot
 * 
 */
public class ArmSubsystem extends ProfiledPIDSubsystem {

  // TODO - set real motor type, real encoder type and various constants
  private final PWMSparkMax m_motor = new PWMSparkMax(ArmConstants.kMotorPort);
  private final Encoder m_encoder = new Encoder(ArmConstants.kEncoderPorts[0], ArmConstants.kEncoderPorts[1]);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      ArmConstants.kSVolts, ArmConstants.kGVolts,
      ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new . */
  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
        setupShuffleboard();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // TODO - implement.
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_motor.setVoltage(output + feedforward);
  }

  @Override
  public double getMeasurement() {
    // TODO - implement
    return m_encoder.getDistance() + ArmConstants.kArmOffsetRads;
  }

  
  /**
   * Raise the Arm by a (small) fixed angle to fine-tune a position
   * 
   */
  public void bumpArmUp() {
    // TODO - implement
    // maybe get the current goal from the controller and increment the angle and reset the goal?
     TrapezoidProfile.State state = m_controller.getGoal();
     double newGoal = state.position + ArmConstants.kArmBumpIncrementRad;
     if ( Math.toDegrees(newGoal) > ArmConstants.kMaxArmAngleDeg) newGoal = Math.toRadians(ArmConstants.kMaxArmAngleDeg);
     setGoal(newGoal);
  }

  /**
   * Lower the Arm by a (small) fixed angle to fine-tune a position
   * 
   */
  public void bumpArmDown() {
    // TODO - implement
    TrapezoidProfile.State state = m_controller.getGoal();
    double newGoal = state.position - ArmConstants.kArmBumpIncrementRad;
    if ( Math.toDegrees(newGoal) < ArmConstants.kMinArmAngleDeg) newGoal = Math.toRadians(ArmConstants.kMinArmAngleDeg);
    setGoal(newGoal);
  }
  private void setupShuffleboard() {
    // use Shuffleboard to facilitate controller param tuning
    ShuffleboardTab arm = Shuffleboard.getTab("Arm");
    try {
      arm.add("Arm Control PID", m_controller);
      arm.add("Arm Control FeedForward", m_feedforward);  // turns out this class doesn't implement Sendable
      
    } catch (Exception e) {// eat it.  for some reason it fails if the tab exists
    }
  }



}
