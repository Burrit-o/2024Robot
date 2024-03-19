// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  private final CANSparkMax LeftLiftMotor, RightLiftMotor;
  private PIDController LiftSetpoint;
  private final TimeOfFlight ToF, BackupToF;
  private final DigitalInput TopLim;
  private final DigitalInput BottomLim;
  private double kp;
  private double ki;
  private double kd;
  private double height;
  private double speed;

  /** Creates a new Lift. */
  public Lift() {
    LeftLiftMotor = new CANSparkMax(LiftConstants.LeftLiftMotor, MotorType.kBrushless);
    RightLiftMotor = new CANSparkMax(LiftConstants.RightLiftMotor, MotorType.kBrushless);

    LeftLiftMotor.setIdleMode(IdleMode.kBrake);
    RightLiftMotor.setIdleMode(IdleMode.kBrake);


    ToF = new TimeOfFlight(LiftConstants.ToFSensor);
    BackupToF = new TimeOfFlight(LiftConstants.BackupToFSensor);

    //meanHeight = (LiftHeight + BackupLiftHeight)/2;
    TopLim = new DigitalInput(3);
    BottomLim = new DigitalInput(2);



  }

  public double getCommandedHeight() {
    return LiftSetpoint.getSetpoint();
  }

  public void setLift(double speed) {
    if (speed < 0) {
      if (!TopLim.get()) {
          LeftLiftMotor.set(0);
          RightLiftMotor.set(0);   
      } else {
          LeftLiftMotor.set(speed);
          RightLiftMotor.set(speed);      }
  } else {
      if (!BottomLim.get()) {
          LeftLiftMotor.set(0);
          RightLiftMotor.set(0);      
      } else {
          LeftLiftMotor.set(speed);
          RightLiftMotor.set(speed);      
      }
    }
  }


  public void runLiftSetpoint() {
    setLift(-LiftSetpoint.calculate(currentHeight()));
  }

  public void setLiftPID(LiftConstants.Setpoint m_Setpoint) {
    //    LiftSetpoint = new PIDController(.003, 0.00225, 0.000075);

    LiftConstants.Setpoint setpoint = m_Setpoint;
    switch (setpoint) {
        case AMP: kp = 0.00287; ki = 0.000875; kd = 0.00007; height = LiftConstants.AmpHeight; speed = .225 ;
      break;
        // case SPEAKER: kp = 0.00315; ki = 0.001125; kd = 0.000075; height = LiftConstants.SpeakerHeight ;
        case SPEAKER: kp = 0.00475; ki = 0.00115; kd = 0.000085; height = LiftConstants.SpeakerHeight; speed = 1 ;
      break;
        case STOW: kp = 0; ki = 0; kd = 0; height = LiftConstants.Stow; speed = 0;
      break;
        case PICKTOP : kp = 0; ki = 0; kd = 0; height = LiftConstants.ClimbTop; speed = 0 ;
      break;
        case PICKBOTTOM: kp = 0; ki = 0; kd = 0; height = LiftConstants.ClimbBottom; speed = 0  ;
      break;
        case PICKUP: kp = .002; ki = 0.0003; kd = 0; height = LiftConstants.PickupHeight; speed = 0  ;
      break;
        case SPEAK4FT: kp = 0.0045; ki = 0.0035; kd = 0.0001125; height = LiftConstants.Speaker4ft; speed = 1 ;
      break;
        case SPEAKPickupSide: kp = 0.0045; ki = 0.0035; kd = 0.0001125; height = LiftConstants.SpeakSidePickupSpot; speed = 1 ;
      break;
        default:kp = 0; ki = 0; kd = 0; height = LiftConstants.Stow; speed = 0 ;
    } 
    LiftSetpoint = new PIDController(kp, ki, kd);
    LiftSetpoint.setSetpoint(height);  
  }

  public boolean atSetpoint() {
    int tolerance = 5;
    if(currentHeight() < getCommandedHeight() + tolerance && currentHeight() > getCommandedHeight() - tolerance) {
      return true;
    }
    return false;
  }
  
  public double currentHeight() {
   /*  if (LiftHeight > meanHeight + BackupToF.getRangeSigma() ||  LiftHeight < meanHeight - BackupToF.getRangeSigma())

     {return BackupLiftHeight;}

    else if (BackupLiftHeight > meanHeight + ToF.getRangeSigma() ||  BackupLiftHeight < meanHeight - ToF.getRangeSigma())

     {return LiftHeight;}

    else {return meanHeight;}*/
    return (BackupToF.getRange()+ToF.getRange())/2;
  }

  public double ShootSpeed(){
    return speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LiftHeight", ToF.getRange()+30);
    SmartDashboard.putNumber("BackupLiftHeight", BackupToF.getRange());
    SmartDashboard.putNumber("MeanHeight", currentHeight());
  }
}
