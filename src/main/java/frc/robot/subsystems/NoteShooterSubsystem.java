// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

//  private TalonFX m_motor = new TalonFX(ShooterConstants.kShooterCanbusID, "rio");
  private SparkFlex m_motor = new SparkFlex(13, MotorType.kBrushless);
  private double motorSpeed = ShooterConstants.kShooterSpeed;
  private GenericEntry nt_shooterspeed;
  /** Creates a new VisionSubsystem. */
  public NoteShooterSubsystem() {
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

  public double getSpeed(){
    return nt_shooterspeed.getDouble(0.1);
  }
  public void spinUp() {
    // start the motor at some pre-defined constant speed
    m_motor.set(getSpeed());
  }

  private void setupShuffleboard(){

    ShuffleboardTab shootTab = Shuffleboard.getTab("shooter");
    nt_shooterspeed = shootTab.addPersistent("ShooterSpeed",0.0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min",-1.0,  "max",0.0))
    .getEntry();
  }
  public void stop() {
    m_motor.set(0.0);
  }

  public Command SpinCommand(){
    return new StartEndCommand(() -> this.spinUp(), () -> this.stop());
  }
}
