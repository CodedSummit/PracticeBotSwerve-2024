// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLedSubsystem extends SubsystemBase {
  private int m_length;
  private AddressableLEDBuffer m_buffer;
  private AddressableLED m_led;
  private ArrayList<Integer> m_colors = new ArrayList<>();
  private ArrayList<Integer> m_topHalfColors = new ArrayList<>();
  
  public AddressableLedSubsystem(int length, int port) {
    m_led = new AddressableLED(port);
    m_length = length;
    m_buffer = new AddressableLEDBuffer(m_length);
    m_led.setLength(m_length);
    m_led.setData(m_buffer);
    m_led.start();

  }

  @Override
  public void periodic() {
  }

  //This one is only used for the setting bottom and top halves methods
  private void appendItem(){
    int counter = 0;
    for(int i=0; i<m_buffer.getLength(); i++){
      if(i > (int) m_buffer.getLength()/2){
        m_topHalfColors.add(counter);
      }
      m_colors.add(counter);
      counter++;
    }
  }

  /*public void setLengthStrip(int stripLength, AddressableLED led){
    //Try to only call this one once.
    led.setLength(stripLength);
    m_length = stripLength;
  }*/

  public void setStripRed(){
setStripColor(255,0,0);
  }

  public void setStripOrange(){
    setStripColor(255,125,0);
  }

  public void setStripYellow(){
    setStripColor(255,255,0);
  }

  public void setStripGreen(){
    setStripColor(0,255,0);
  }

  public void setStripBlue(){
    setStripColor(0,0,255);
  }

  public void setStripPurple(){
    setStripColor(255,0,255);
  }

  public void setStripColor(int r, int g, int b){
    for(int i = 0; i<m_buffer.getLength(); i++){
      m_buffer.setRGB(i,r,g,b);
    }
    m_led.setData(m_buffer);
  }

  //This one may not work, because it sets the LED's colors to black.
  public void setStripOff(){
    setStripColor(0,0,0);
  }

  // Both setStripLowerHalf and setStripUpperHalf are experimental and may not work as expected (or at all).
  public void setStripLowerHalf(int r, int g, int b){
    for(int i = 0; i<m_buffer.getLength() - (int) m_buffer.getLength()/2; i++){
      m_buffer.setRGB(i,r,g,b);
    }
  }

  public void setStripUpperHalf(int r1, int g1, int b1){
    for(int i = 0; i<m_topHalfColors.get(i); i++){
      m_buffer.setRGB(i,r1,r1,b1);
    }
  }

  public int randomNumber(int min, int max){
    int number;
    number = (int) Math.random();
    if (number > max){
      number = max;
      return number;
    } else if (number < min){
      number = min;
      return number;
    }
    return number;
  }
  
  public void setStripRandom(){
    for(int i = 0; i<m_buffer.getLength(); i++){
      m_buffer.setRGB(i, randomNumber(0,255), randomNumber(0,255) , randomNumber(0,255));
    }
  }
}

