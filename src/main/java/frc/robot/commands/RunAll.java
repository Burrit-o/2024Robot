// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.IPFSSub;


public class RunAll extends Command {
  private final Subsystem m_subsystem;
  /** Creates a new RunAll. */
  public RunAll(IPFSSub subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ((IPFSSub) m_subsystem).Feed(0.25);
    ((IPFSSub) m_subsystem).Intake(0.5);
    ((IPFSSub) m_subsystem).Shoot(0.75);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     ((IPFSSub) m_subsystem).Feed(0);
    ((IPFSSub) m_subsystem).Intake(0);
    ((IPFSSub) m_subsystem).Shoot(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
