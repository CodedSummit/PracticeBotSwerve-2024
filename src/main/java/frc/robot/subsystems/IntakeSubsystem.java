// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem responsible for picking up Notes
 * 
 * Knows if it holds a Note or not
 * Can release a Note to other systems (or maybe just know that it was taken)
 * 
 * 
 */
public class IntakeSubsystem extends SubsystemBase {

  boolean m_hasNote = false; // is known to be holding a Note

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

}
