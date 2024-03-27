// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import frc.robot.LEDs;
import frc.robot.RobotContainer;
import frc.robot.Constants.IPFSConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LEDConstants.statusLED;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.LEDs;



public class IPFSSub extends SubsystemBase {
  private final CANSparkMax TLShooterMotor;
  private final CANSparkMax Feeder;
  private final CANSparkMax Feeder2;
  private final CANSparkMax TRShooterMotor;
  private final CANSparkMax BLShooterMotor;
  private final CANSparkMax BRShooterMotor;
  private final CANSparkMax IntakeMotorTop;
  private final CANSparkMax IntakeMotorBottom;
  
  
  private final RelativeEncoder TLEncoder; 
  private final RelativeEncoder TREncoder;
  private final RelativeEncoder BLEncoder;
  private final RelativeEncoder BREncoder;

  private LEDs m_LEDs;

  public final DigitalInput PickupSensor;  
  //private final LEDs m_LEDs;
  private boolean haveNOTESet = false;
  private boolean seeNOTESet = false;
  
  private int counter = 0;

  /** Creates a new IPFSSub. */
  public IPFSSub(LEDs leds) {
  m_LEDs = leds;
  TLShooterMotor = new CANSparkMax(IPFSConstants.TLShooterMotor, MotorType.kBrushless);
  TRShooterMotor = new CANSparkMax(IPFSConstants.TRShooterMotor, MotorType.kBrushless);
  TLShooterMotor.setInverted(true);
  TLShooterMotor.setIdleMode(IdleMode.kCoast);
  TRShooterMotor.setIdleMode(IdleMode.kCoast);

  BLShooterMotor = new CANSparkMax(IPFSConstants.BLShooterMotor, MotorType.kBrushless);
  BRShooterMotor = new CANSparkMax(IPFSConstants.BRShooterMotor, MotorType.kBrushless);
  BRShooterMotor.setInverted(true);
  BLShooterMotor.setIdleMode(IdleMode.kCoast);
  BRShooterMotor.setIdleMode(IdleMode.kCoast);

  Feeder = new CANSparkMax(IPFSConstants.LFeederMotor, MotorType.kBrushless);
  Feeder2 = new CANSparkMax(IPFSConstants.RFeederMotor, MotorType.kBrushless);
  Feeder.setInverted(true);
  Feeder.setIdleMode(IdleMode.kBrake);
  Feeder2.setIdleMode(IdleMode.kBrake);

  
  IntakeMotorTop = new CANSparkMax(IPFSConstants.PickupMotorTop, MotorType.kBrushless);
  IntakeMotorBottom = new CANSparkMax(IPFSConstants.PickupMotorBottom, MotorType.kBrushless);
  IntakeMotorTop.setInverted(true);
  IntakeMotorTop.setIdleMode(IdleMode.kBrake);
  IntakeMotorBottom.setIdleMode(IdleMode.kBrake);  

  
  PickupSensor = new DigitalInput(4);

  TLEncoder = TLShooterMotor.getEncoder();
  TREncoder = TRShooterMotor.getEncoder();
  BLEncoder = BLShooterMotor.getEncoder();
  BREncoder = BRShooterMotor.getEncoder();

  }

  public double TLVelocity() {
    return TLEncoder.getVelocity();
  }
  
  public double TRVelocity() {
    return TREncoder.getVelocity();
  }

  public double BLVelocity() {
    return BLEncoder.getVelocity();
  }
  
  public double BRVelocity() {
    return BREncoder.getVelocity();
  }

  public void Shoot(double speed) {
    //left shooter motor temporarily off due to spark max issues
    TLShooterMotor.set(speed);
    TRShooterMotor.set(speed);
    BLShooterMotor.set(speed);
    BRShooterMotor.set(speed);
  }
  
  public void Feed(double speed) {
    Feeder.set(speed);
    Feeder2.set(speed);
  }

  public void Intake(double speed) {
   IntakeMotorTop.set(speed);
   IntakeMotorBottom.set(speed);
  }


  public boolean haveNote() {
    return PickupSensor.get();
  }

  public boolean canSeeNote() {
  // If we can see a NOTE, return true
  if(NetworkTableInstance.getDefault().getTable(VisionConstants.kPickupLimelightNetworkTableName).getEntry("tx").getDouble(0) != 0) {
    counter = 0;
    return true;
  }
  // Reduce noise
  else {
    if(counter < 3) {
      counter = counter + 1;
      return true;
    }
  }
  return false;
}

@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("PickupSensor", PickupSensor.get());
    SmartDashboard.putBoolean("CanSeeNote", canSeeNote());
    // Only change LED color when state changes (not every clock cycle)
    if(!haveNote() && !haveNOTESet) {
      m_LEDs.signal(statusLED.STRIP1, ledMode.GREEN);
      m_LEDs.signal(statusLED.STRIP3, ledMode.GREEN);
      haveNOTESet = true;
    } 
    else if (canSeeNote() && !seeNOTESet && haveNote()) {
      m_LEDs.signal(statusLED.STRIP1, ledMode.YELLOW);
      m_LEDs.signal(statusLED.STRIP3, ledMode.YELLOW);
      seeNOTESet = true;
    }
    else if (haveNote() && !canSeeNote()) {
      m_LEDs.signal(statusLED.STRIP1, ledMode.RED);
      m_LEDs.signal(statusLED.STRIP3, ledMode.RED);
      haveNOTESet = false;
      seeNOTESet = false;
    }
  }
}
