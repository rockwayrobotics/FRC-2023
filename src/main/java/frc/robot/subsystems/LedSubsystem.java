// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LED.modes;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  public class LED{
    private int r;
    private int b;
    private int g;
    public LED(int m_r, int m_b, int m_g){
      r = m_r;
      b = m_b;
      g = m_g;
    }

    public int[] get_vals(){
      int[] return_arr = {this.r, this.b, this.g};
      return return_arr;
    }
  }

  private void reset(){
    for (int i=0; i < m_ledBuffer.getLength(); i++){
      previous_led.add(new LED(0,0,0));
    }
  }

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private modes m_mode;
  private int counter;
  private int counter2;
  private ArrayList<LED> previous_led = new ArrayList<LED>() ;

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

  private double gradient_helper(double value, boolean invert){
    double sin_wave = Math.sin(Math.PI * 2 * counter / 255);
    if (invert){
      return (value + (-sin_wave * 255));
    }
    else{
      return (value + (sin_wave * 255));
    }
  }

  private void red_green_breathe_gradient() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      double r = gradient_helper(i, false);
      double g = 255 - gradient_helper(i, true);
      double b = 0;
      m_ledBuffer.setRGB(i, (int)r, (int)g, (int)b);
    }
    counter++;
  }

  private void single_red_dot(){
    for (int i = 0; i < m_ledBuffer.getLength(); i++){
      double r = 0;
      if (i == counter % m_ledBuffer.getLength()){
        r = 255;
      }
      double g = 0;
      double b = 0;
      m_ledBuffer.setRGB(i, (int)r, (int)g, (int)b);
    }
    counter++;
  }

  private void apply_sequence(){
    for (int i=0; i < m_ledBuffer.getLength(); i++){
      LED current_led = previous_led.get(i);
      int[] out = current_led.get_vals();
      m_ledBuffer.setRGB(i, out[0], out[1], out[2]);
    }
    }

  private void move_sequence(){
    LED last_led = previous_led.get(previous_led.size() - 1);
    previous_led.remove(previous_led.size() - 1);
    previous_led.add(0, last_led);
    apply_sequence();
  }

  private void gen_chasing_dots(){
    previous_led.set((int)previous_led.size() / 2, new LED(0,255,0));
    previous_led.set(0, new LED(255, 0, 0));
  }



  private void building_red_dots(){
    //TODO: Implement
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

  private void nonbinaryFlag() {
    for (var i = 0; i < Math.round((m_ledBuffer.getLength()/3)); i++) {
      m_ledBuffer.setRGB(i, 255, 255, 0);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/3)); i < Math.round((m_ledBuffer.getLength()/5)*3); i++) {
      m_ledBuffer.setRGB(i, 192, 192, 192);
    }
    for (var i = Math.round((m_ledBuffer.getLength()/3)); i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 0, 53, 196);
    }
  }

  private void aroaceFlag(){
      for (var i = 0; i < Math.round((m_ledBuffer.getLength()/5)); i++) {
        m_ledBuffer.setRGB(i, 255, 150, 0);
      }
      for(var i = 0; i < Math.round(m_ledBuffer.getLength() / 5); i++){
        m_ledBuffer.setRGB(i, 255, 200, 0);
      }
      for (var i = Math.round((m_ledBuffer.getLength()/5)); i < Math.round((m_ledBuffer.getLength()/5)*3); i++) {
        m_ledBuffer.setRGB(i, 192, 192, 192);
      }
      for(var i = 0; i < Math.round(m_ledBuffer.getLength() / 5); i++){
        m_ledBuffer.setRGB(i, 0, 170, 255);
      }
      for (var i = Math.round((m_ledBuffer.getLength()/5)); i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 50, 255);
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
    counter = 0;
    counter2 = 0;
    reset();
    switch(m_mode) {
      case ChasingDots :
        gen_chasing_dots();
      default :
        reset();
    }
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
        case RedGreenBreatheGradient:
          red_green_breathe_gradient();
          break;
        case SingleRedDot:
          single_red_dot();
          break;
        case Enby:
          nonbinaryFlag();
          break;
        case AroAce:
          aroaceFlag();
          break;
        case ChasingDots:
          move_sequence();
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