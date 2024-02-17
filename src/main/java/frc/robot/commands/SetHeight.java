// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IPFSSub;

public class SetHeight extends Command {
  private final IPFSSub m_subsystem;
  private final double m_setpoint;
  /** Creates a new SetHeight. */
  public SetHeight(IPFSSub subsystem, double setpoint) {
      m_subsystem = subsystem;
      m_setpoint = setpoint;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLiftSetpoint(m_setpoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.runLiftSetpoint();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
if (Math.abs(m_setpoint-m_subsystem.CurrentHeight()) <= 20) {
      return true;
    } else {
      return false;
    } 
   }
}
