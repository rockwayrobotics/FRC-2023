// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;
import java.util.Random;
import java.util.random.*;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants.LED_CONSTANTS;
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
    // previous_led = new ArrayList<LED>();
    System.out.println(previous_led.size());
    for (int i=0; i < m_ledBuffer.getLength(); i++){
      previous_led.add(new LED(0,0,0));
    }
    System.out.println(previous_led.size());
  }

  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  private int m_rainbowFirstPixelHue;
  private modes m_mode;
  private int counter;
  private int counter2;
  private ArrayList<LED> previous_led = new ArrayList<LED>();
  private ArrayList<LED> full_sequence = new ArrayList<LED>();
  private modes[] possible_patterns = {modes.SingleRedDot, modes.ChasingDots, modes.Rainbow, modes.PiSequence, modes.RedGreenBreatheGradient};

  public LedSubsystem(
    int m_ledInt,
    int m_ledBufferInt
  ) {

    m_led = new AddressableLED(m_ledInt);
    m_ledBuffer = new AddressableLEDBuffer(m_ledBufferInt);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);

    m_led.start();

    setMode(pick_random_pattern());

    // m_mode = pick_random_pattern();
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
    // System.out.println();
    // for (int i=0; i < previous_led.size(); i++){
    //   System.out.print(previous_led.get(i).r + ",");
    // }
    // System.out.print("\n");
    System.out.println(previous_led.size());
  }

  private void move_sequence_from_full(){
    int endpoint = counter % full_sequence.size() + m_ledBuffer.getLength();
    if (endpoint >= full_sequence.size()){
      // previous_led = (ArrayList<LED>)full_sequence.subList(counter, full_sequence.size() - 1);
      for (LED led : full_sequence.subList(counter, full_sequence.size() - 1) ){
        previous_led.add(led);
      }
      for (LED led :  full_sequence.subList(0, endpoint % full_sequence.size())){
        previous_led.add(led);
      }
      // previous_led.addAll(previous_led.size() - 1, full_sequence.subList(0, endpoint % full_sequence.size()));
    }
    else{
      // previous_led = (ArrayList<LED>)full_sequence.subList(counter, counter + m_ledBuffer.getLength());
      for (LED led : full_sequence.subList(counter, counter + m_ledBuffer.getLength())){
        previous_led.add(led);
      }
    }
    apply_sequence();
    System.out.println(previous_led);
  }


  private void gen_chasing_dots(){
    LED red = new LED(255,0,0);
    LED green = new LED(0,255,0);
    LED blue = new LED(0,0,255);
    int spacing = ((int)m_ledBuffer.getLength() / 6);
    // previous_led.set((int)m_ledBuffer.getLength() / 2, new LED(0,255,0));
    previous_led.set(0, red);
    previous_led.set(spacing * 1, green);
    previous_led.set(spacing * 2, blue);
    previous_led.set(spacing * 3, red);
    previous_led.set(spacing * 4, green);
    previous_led.set(spacing * 5, blue);
  }

  private void gen_pi_sequence(){
    String[] pi_arr = LED_CONSTANTS.PI_STRING.split(" ", 0);
    for (int i=0; i < m_ledBuffer.getLength(); i++){
      full_sequence.add(new LED(0,0,0));
    }
    for (String digit: pi_arr){
      for (int i=0; i < Integer.valueOf(digit); i++){
        full_sequence.add(new LED(0,255,0));
      }
      full_sequence.add(new LED(0,0,0));
    }
    full_sequence.remove(full_sequence.size() - 1);
  }

  private void exciting_monochrome(String color){
    LED base_led;
    switch (color) {
      case "r" :
        base_led = new LED(255,0,0);
      case "b" :
        base_led = new LED(0, 255, 0);
      case "g" :
       base_led = new LED(0, 0, 255);
      case "y" :
        base_led = new LED(255, 255, 0);
      case "c" :
        base_led = new LED(0, 255, 255);
      case "m" :
        base_led = new LED(255, 0, 255);
      case "w" :
        base_led = new LED(255, 255, 255);
      default :
        base_led = new LED(255,255,255);
    for (int i=255; i >= 100; i--){
      LED current_led = base_led;
      full_sequence.add(current_led);
      if (base_led.r != 0){
        base_led.r--;
      }
      if (base_led.g != 0){
        base_led.g--;
      }
      if (base_led.b != 0){
        base_led.b--;
      }
    }

    }
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

  private int get_rand_number(int min, int max){
    var myNumber = (int) ((Math.random() * (max - min) + min));
    System.out.println("random number selected: " + myNumber);

    return myNumber;
  }

  public modes pick_random_pattern(){
    var myPattern = possible_patterns[get_rand_number(0, possible_patterns.length - 1)];

    System.out.println("Selected pattern: " + myPattern);

    return myPattern;
  }

  public void setMode(modes mode) {
    m_mode = mode;
    counter = 0;
    counter2 = 0;
    full_sequence = new ArrayList<LED>();
    previous_led = new ArrayList<LED>();
    reset();
    switch(m_mode) {
      case ChasingDots :
        gen_chasing_dots();
      case PiSequence :
        gen_pi_sequence();
      case ExcitingMonochromeM :
        exciting_monochrome("m");
      case ExcitingMonochromeY :
        exciting_monochrome("y");
      default :
        break;
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
        case PiSequence:
          move_sequence_from_full();
          break;
        case ExcitingMonochromeAny:
          move_sequence_from_full();
          break;
        case ExcitingMonochromeM:
          move_sequence_from_full();
          break;
        case ExcitingMonochromeY:
          move_sequence_from_full();
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