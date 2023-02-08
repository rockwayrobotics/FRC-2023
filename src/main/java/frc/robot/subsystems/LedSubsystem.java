// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  public int m_mode;

  public LedSubsystem(
    int m_ledInt,
    int m_ledBufferInt
  ) {

    m_led = new AddressableLED(m_ledInt);
    m_ledBuffer = new AddressableLEDBuffer(m_ledBufferInt);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();

    m_mode = 0;

  }

  private void rainbow() {
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
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 128, 0, 0);
    }
  }

  private void green() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 0, 128, 0);
    }
  }

  private void blue() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 0, 0, 128);
    }
  }

  private void yellow() {
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 128, 128, 0);
    }
  }

  private void funnyProper() {
    for (var i = 0; i < Math.round((m_ledBuffer.getLength()/5)*2); i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/5)*2); i < Math.round((m_ledBuffer.getLength()/5)*3); i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 156, 78, 151);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/5)*3); i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 0, 53, 196);
    }
  }

  private void funny2() {
    for (var i = 0; i < 12; i++) {
        // Sets the specified LED to the HSV values for red
        m_ledBuffer.setRGB(i, 61, 157, 240);
    }
    for (var i = 12; i < 24; i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = 24; i < 36; i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 192, 192, 192);
    }
    for (var i = 36; i < 48; i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 215, 0, 113);
    }
    for (var i = 48; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the HSV values for red
      m_ledBuffer.setRGB(i, 61, 157, 250);
    }
  }


  public void setMode(int mode) {
    m_mode = mode;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch(m_mode) {
        case 1:
            green();
            break;
        case 2:
            red();
            break;
        case 3:
            funnyProper();
            break;
        case 4:
            funny2();
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
