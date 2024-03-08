// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/**
 * Models the mechanism that shoots Notes
 * 
 * Has a Note (or not) from IntakeSubsystem
 * (TBD - this may be fixed prior to match) Shooter can adjust height - has an absolute encoder and a motor to change the
 * height.
 * The height/angle adjustment will need a PID controller. (TBD - may be fixed prior to match and not moveable)
 * May not directly know it has a note
 * It'll spin up prior to the Intake feeding it into the shooter.  Command or initiator is responsible 
 * for managing any spinup time
 * Spins at an unregulated velocity- a fixed rate determined a priori for expected target distance.
 * 
 */
public class NoteShooterSubsystem extends SubsystemBase {

  private TalonFX m_motor = new TalonFX(ShooterConstants.kShooterCanbusID, "rio");

  /** Creates a new VisionSubsystem. */
  public NoteShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void spinUp() {
    // start the motor at some pre-defined constant speed
    m_motor.set(ShooterConstants.kShooterSpeed);
  }

  public void stop() {
    m_motor.set(0.0);
  }
}
