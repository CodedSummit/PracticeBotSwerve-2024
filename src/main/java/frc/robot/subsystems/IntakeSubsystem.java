// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * The subsystem responsible for picking up Notes
 * 
 * Knows if it holds a Note or not
 * Can release a Note to other systems (or maybe just know that it was taken)
 * When releasing the Note to the Shooter subsystem, that triggers the shot (i.e. Shooter 
 * does not hold the Note)
 * 
 * 
 */
public class IntakeSubsystem extends SubsystemBase {

  boolean m_hasNote = false; // is known to be holding a Note
  CANSparkMax m_motor  = new CANSparkMax(IntakeConstants.kIntakeCanbusID, MotorType.kBrushless);
  // TODO - use the beam break to know a Note is picked up

  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public boolean hasNote() {
    // TODO - implement. Return t/f if a Note is in the handler
    return m_hasNote;
  }

  /**
   * Starts up the intake motor

  */
  public void setSpeed() {
    // TODO - implement
    m_motor.set(IntakeConstants.intakeSpeed);
  }

  /**
   * Stops the intake motor
   */
  public void stop() {
    // TODO -implement
    m_motor.set(0.0);
  }

}
