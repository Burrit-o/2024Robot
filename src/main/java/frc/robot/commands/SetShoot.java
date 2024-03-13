// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

import frc.robot.Constants.LiftConstants;

public class SetShoot extends Command {
  Lift lift;
  LiftConstants.Setpoint  setpoint;
  Command command;
  /** Creates a new SetShoot. */
  public SetShoot(Lift subsytem, Command Command, LiftConstants.Setpoint Setpoint) {
    lift = subsytem;
    setpoint = Setpoint;
    command = Command;
    addRequirements(lift);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lift.setLiftPID(setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    command.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(lift.currentHeight()-lift.getCommandedHeight()) <= 5){
      return true;
    }else{
    return false;
    }
  }
}
