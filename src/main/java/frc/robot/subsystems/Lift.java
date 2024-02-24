// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;

public class Lift extends SubsystemBase {
  private final CANSparkMax LeftLiftMotor, RightLiftMotor;
  private PIDController LiftSetpoint;
  private final TimeOfFlight ToF, BackupToF;
  private final double meanHeight;
  private final double LiftHeight;
  private final double BackupLiftHeight;


  /** Creates a new Lift. */
  public Lift() {
    LeftLiftMotor = new CANSparkMax(LiftConstants.LeftLiftMotor, MotorType.kBrushless);
    RightLiftMotor = new CANSparkMax(LiftConstants.RightLiftMotor, MotorType.kBrushless);

    LeftLiftMotor.setIdleMode(IdleMode.kBrake);
    RightLiftMotor.setIdleMode(IdleMode.kBrake);

    LiftSetpoint = new PIDController(.08, .008, 0);

    ToF = new TimeOfFlight(LiftConstants.ToFSensor);
    BackupToF = new TimeOfFlight(LiftConstants.BackupToFSensor);
    LiftHeight = ToF.getRange();
    BackupLiftHeight = BackupToF.getRange();

    meanHeight = (LiftHeight + BackupLiftHeight)/2;




  }


  public void setLift(double speed) {
    LeftLiftMotor.set(speed);
    RightLiftMotor.set(speed);
  }

  public void setLiftSetpoint(double setpoint){
    LiftSetpoint.setSetpoint(setpoint);
  }

  public void runLiftSetpoint() {
    setLift(LiftSetpoint.calculate(currentHeight()));
  }

  public double currentHeight() {
    if (LiftHeight > meanHeight + BackupToF.getRangeSigma() ||  LiftHeight < meanHeight - BackupToF.getRangeSigma())

     {return BackupLiftHeight;}

    else if (BackupLiftHeight > meanHeight + ToF.getRangeSigma() ||  BackupLiftHeight < meanHeight - ToF.getRangeSigma())

     {return LiftHeight;}

    else {return meanHeight;}
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LiftHeight", LiftHeight);
    SmartDashboard.putNumber("BackupLiftHeight", BackupLiftHeight);

  }
}
