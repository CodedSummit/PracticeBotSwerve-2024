// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Models the mechanism that shoots Notes
 * 
 * Has a Note (or not) from NoteHandlerSS
 * Shooter can adjust height - has an absolute encoder and a motor to change the
 * height.
 * The height/angle adjustment will need a PID controller.
 * May not directly know it has a note
 * It'll spin up prior to the Intake feeding it into the shooter.
 */
public class NoteShooterSubsystem extends SubsystemBase {

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

}
