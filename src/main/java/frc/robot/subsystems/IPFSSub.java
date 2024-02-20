// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.PickupConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.playingwithfusion.TimeOfFlight;




public class IPFSSub extends SubsystemBase {
  private final CANSparkMax TLShooterMotor;
  private final CANSparkMax Feeder;
  private final CANSparkMax TRShooterMotor;
  private final CANSparkMax BLShooterMotor;
  private final CANSparkMax BRShooterMotor;
  private final CANSparkMax IntakeMotor;
  private final CANSparkMax LeftLiftMotor;
  private final CANSparkMax RightLiftMotor;
  
  private final RelativeEncoder TLEncoder; 
  private final RelativeEncoder TREncoder;
  private final RelativeEncoder BLEncoder;
  private final RelativeEncoder BREncoder;

  public final DigitalInput PickupSensor;  
  private final TimeOfFlight LiftHeight;

  public PIDController LiftSetpoint;


  /** Creates a new IPFSSub. */
  public IPFSSub() {
  TLShooterMotor = new CANSparkMax(ShooterConstants.TLShooterMotor, MotorType.kBrushless);
  TRShooterMotor = new CANSparkMax(ShooterConstants.TRShooterMotor, MotorType.kBrushless);
  TRShooterMotor.setInverted(true);
  BLShooterMotor = new CANSparkMax(ShooterConstants.BLShooterMotor, MotorType.kBrushless);
  BRShooterMotor = new CANSparkMax(ShooterConstants.BRShooterMotor, MotorType.kBrushless);
  Feeder = new CANSparkMax(PickupConstants.PFeederMotor, MotorType.kBrushless);
  IntakeMotor = new CANSparkMax(PickupConstants.PickupMotor, MotorType.kBrushless);
  LeftLiftMotor = new CANSparkMax(LiftConstants.LeftLiftMotor, MotorType.kBrushless);
  RightLiftMotor = new CANSparkMax(LiftConstants.RightLiftMotor, MotorType.kBrushless);
  TLShooterMotor.setIdleMode(IdleMode.kBrake);
  TRShooterMotor.setIdleMode(IdleMode.kBrake);
  BLShooterMotor.setIdleMode(IdleMode.kBrake);
  BRShooterMotor.setIdleMode(IdleMode.kBrake);
  Feeder.setIdleMode(IdleMode.kBrake);
  IntakeMotor.setIdleMode(IdleMode.kBrake);
  LeftLiftMotor.setIdleMode(IdleMode.kBrake);
  RightLiftMotor.setIdleMode(IdleMode.kBrake);

  
  PickupSensor = new DigitalInput (1);
  LiftHeight = new TimeOfFlight(LiftConstants.ToFSensor);

  TLEncoder = TLShooterMotor.getEncoder();
  TREncoder = TRShooterMotor.getEncoder();
  BLEncoder = BLShooterMotor.getEncoder();
  BREncoder = BRShooterMotor.getEncoder();

  LiftSetpoint = new PIDController(.08, .008, 0);

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
  }
  
  public void Feed(double speed) {
    Feeder.set(speed);
  }

  public void Intake(double speed) {
   IntakeMotor.set(speed);
  }

  public void setLift(double speed) {
    LeftLiftMotor.set(speed);
    RightLiftMotor.set(speed);
  }

  public void setLiftSetpoint(double setpoint){
    LiftSetpoint.setSetpoint(setpoint);
  }

  public void runLiftSetpoint() {
    setLift(LiftSetpoint.calculate(LiftHeight.getRange()));
  }

  public double CurrentHeight() {
    return LiftHeight.getRange();
  }

  public boolean haveNote() {
    return PickupSensor.get();
  }

    @Override

  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("PickupSensor", PickupSensor.get());

  }
}
