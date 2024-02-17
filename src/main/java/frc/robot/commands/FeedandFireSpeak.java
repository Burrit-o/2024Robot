// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LiftConstants;
import frc.robot.subsystems.IPFSSub;

public class FeedandFireSpeak extends Command {
  private final IPFSSub m_subsystem;
  /** Creates a new FeedandFire. */
  public FeedandFireSpeak(IPFSSub subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLiftSetpoint(LiftConstants.SpeakerHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runLiftSetpoint();
    m_subsystem.Feed(0.167);
    m_subsystem.Shoot(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.Feed(0);
    m_subsystem.Shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(LiftConstants.SpeakerHeight-m_subsystem.CurrentHeight()) <= 20) {
      return true;
    } else {
      return false;
    }
  }
}
