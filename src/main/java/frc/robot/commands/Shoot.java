// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.IPFSSub;
import frc.robot.Constants.LiftConstants;
import edu.wpi.first.wpilibj.Timer;

public class Shoot extends Command {
  /** Creates a new Shoot. */
  IPFSSub Shooter;
  Lift Lift;
  LiftConstants.Setpoint setpoint;
  double speed;
  Timer timer;

  public Shoot(IPFSSub shooter, Lift lift, LiftConstants.Setpoint SETPOINT) {
    Shooter = shooter;
    Lift = lift;
    setpoint = SETPOINT;
    speed = Lift.ShootSpeed();
    addRequirements(Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Lift.setLiftPID(setpoint);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Lift.currentHeight()-Lift.getCommandedHeight()) < 5){
      Shooter.Shoot(speed);
        if (timer.get() >= 1){
          Shooter.Shoot(speed);
          Shooter.Feed(1);
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
    Shooter.Shoot(0);
    Shooter.Feed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() >= 2){
      return true;
    }else{
    return false;
    }
  }
}
