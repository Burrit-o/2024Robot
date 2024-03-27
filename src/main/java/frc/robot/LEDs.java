// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LEDConstants.statusLED;

/** Add your docs here. */
public class LEDs extends SubsystemBase {
private AddressableLEDBuffer m_ledBuffer; 
private AddressableLED m_LEDs;
private ledMode curr_color;
private boolean new_change;
private AddressableLEDBuffer red_ledBuffer;
private AddressableLEDBuffer blue_ledBuffer;
private AddressableLEDBuffer green_ledBuffer;
private AddressableLEDBuffer purple_ledBuffer;
private AddressableLEDBuffer orange_ledBuffer;
private AddressableLEDBuffer yellow_ledBuffer;
private AddressableLEDBuffer zia_ledBuffer;
  
//Constructor for LEDs class
public LEDs(int length, int PWMPort) {
  
  new_change = false;
  //curr_color = ledMode.PURPLE;
  m_ledBuffer = new AddressableLEDBuffer(length);
  m_LEDs = new AddressableLED(LEDConstants.LEDport);

  red_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  blue_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  green_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  purple_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  orange_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  yellow_ledBuffer = new AddressableLEDBuffer(LEDConstants.statusLEDlength);
  zia_ledBuffer = new AddressableLEDBuffer(LEDConstants.ziaLEDlength);

  //Populate the status LED color buffers
  for (var i = 0; i < LEDConstants.statusLEDlength; i++) {
    red_ledBuffer.setRGB(i, 255, 0, 0);
    blue_ledBuffer.setRGB(i, 0, 0, 255);
    green_ledBuffer.setRGB(i, 0, 255, 0);
    purple_ledBuffer.setRGB(i, 135, 0, 211);
    orange_ledBuffer.setRGB(i, 255, 20, 0);
    yellow_ledBuffer.setRGB(i, 255, 255, 0);
  }

  //Populate the ZIA symbol LED buffer with TEAM colors
  for (var i = 0; i < 8; i++) {
    zia_ledBuffer.setRGB(i, 135, 0, 211);
  }
  for (var i = 8; i < LEDConstants.ziaLEDlength; i++) {
    zia_ledBuffer.setRGB(i, 255, 20, 0);
  }

  LED_init();
}


private void myarraycopy(AddressableLEDBuffer small, int srcPos, AddressableLEDBuffer big, int destPos, int copyLen) {
  for (int i = 0; i < copyLen; i++) {
    big.setLED(destPos+i, small.getLED(srcPos+i));
  }
}


public void LED_init() {
  m_LEDs.setLength(m_ledBuffer.getLength());

  //Set default colors on the LED buffer
  signal(statusLED.STRIP1, ledMode.RED);
  signal(statusLED.STRIP2, ledMode.RED);
  signal(statusLED.STRIP3, ledMode.RED);
  myarraycopy(zia_ledBuffer, 0, m_ledBuffer, 45, LEDConstants.ziaLEDlength);

  //Send the default colors to the LEDs and start
  m_LEDs.setData(m_ledBuffer);
  m_LEDs.start();
} 

public void signal (statusLED strip, ledMode color) {
  statusLED m_strip = strip;
  ledMode m_color = color;
  int offset = 0;

  switch (m_strip) {
    case STRIP1: offset = 0;
    break;
    case STRIP2: offset = 15;
    break;
    case STRIP3: offset = 30;
    break;
    default:break;
  }

  switch (m_color) {
    case RED: myarraycopy(red_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength);
    break;
    case GREEN: myarraycopy(green_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength);
    break;
    case YELLOW: myarraycopy(yellow_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength); 
    break;
    case ORANGE: myarraycopy(orange_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength); 
    break;
    case PURPLE: myarraycopy(purple_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength); 
    break;
    case BLUE: myarraycopy(blue_ledBuffer, 0, m_ledBuffer, offset, LEDConstants.statusLEDlength); 
    break;
    default: break;
  }

  new_change = true;
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (new_change) {
      m_LEDs.setData(m_ledBuffer);
      new_change = false;
    }
  }
}