// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LED.modes;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private modes m_mode;

  public LedSubsystem(
    int m_ledInt,
    int m_ledBufferInt
  ) {

    m_led = new AddressableLED(m_ledInt);
    m_ledBuffer = new AddressableLEDBuffer(m_ledBufferInt);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();

    m_mode = modes.Rainbow;

  }

  private void rainbow() {
    // System.out.println("Rainbowing");

    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 64);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
  }

  private void red() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 0);
    }
  }

  private void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 128, 0);
    }
  }

  private void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 0, 128);
    }
  }

  private void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 128, 0);
    }
  }

  private void purple() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 128, 0, 128);
    }
  }

  private void biFlag() {
    for (var i = 0; i < Math.round((m_ledBuffer.getLength()/5)*2); i++) {
      m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/5)*2); i < Math.round((m_ledBuffer.getLength()/5)*3); i++) {
      m_ledBuffer.setRGB(i, 156, 78, 151);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/5)*3); i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 53, 196);
    }
  }

  private void transFlag() {
    for (var i = 0; i < 12; i++) {
      m_ledBuffer.setRGB(i, 51, 147, 240);
    }
    for (var i = 12; i < 24; i++) {
      m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = 24; i < 36; i++) {
      m_ledBuffer.setRGB(i, 192, 192, 192);
    }
    for (var i = 36; i < 48; i++) {
      m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = 48; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 51, 147, 250);
    }
  }


  public void setMode(modes mode) {
    m_mode = mode;
    System.out.println("Set LED to: " + mode);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(m_mode) {
        case Green:
            green();
            break;
        case Red:
            red();
            break;
        case Blue:
            blue();
            break;
        case Yellow:
            yellow();
            break;
        case Purple:
            purple();
            break;
        case Bi:
            biFlag();
            break;
        case Trans:
            transFlag();
            break;
        default:
            rainbow();
    }
    // Set the LEDs
    m_led.setData(m_ledBuffer);
  }
  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation

  }
}
