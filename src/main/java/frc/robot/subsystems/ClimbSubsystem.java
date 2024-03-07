// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Models the mechanism that is used to climb at the end of a match.
 * 
 * This will need to 'extend' to push the hooks up, and then 'climb' which will
 * pull the robot up.
 * lowering is just technically 'extending'. after the match, robots are not
 * activated again.
 * We'll either need to lift the robot off the chain (likely the best answer) or
 * use a ratchet to manually
 * adjust the motor (it's a worm gearbox, so not backdrivable).
 */
public class ClimbSubsystem extends SubsystemBase {

  public ClimbSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Climb (raise the robot on the hook). Assumes the arm/hook has already been
   * positioned correctly.
   */
  public void climb() {
    // TODO - implement
  }
}
