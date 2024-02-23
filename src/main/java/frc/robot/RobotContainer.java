// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IPFSSub;
import frc.robot.subsystems.Lift;
import frc.robot.commands.FeedandFireAmp;
import frc.robot.commands.FeedandFireSpeak;
import frc.robot.commands.Pickup;
import frc.robot.commands.SetHeight;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IPFSSub m_IPFSSub;
  private final Lift m_Lift;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_IPFSSub = new IPFSSub();
    m_Lift = new Lift();
    configureBindings();
    m_Lift.setDefaultCommand(new SetHeight(m_Lift, LiftConstants.Stow));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

      
    Trigger rBumper = m_operatorController.rightBumper();
    rBumper.whileTrue(new FeedandFireSpeak(m_IPFSSub));
    rBumper.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.SpeakerHeight)));

    Trigger lBumper = m_operatorController.leftBumper();
    lBumper.whileTrue(new FeedandFireAmp(m_IPFSSub));
    lBumper.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.AmpHeight)));
    
    Trigger aButton = m_operatorController.a();
    aButton.whileTrue(new Pickup(m_IPFSSub));
    aButton.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.PickupHeight)));


    Trigger DPadUp = m_operatorController.povUp();
    Trigger DPadDown = m_operatorController.povDown();
    Trigger DPadLeft = m_operatorController.povLeft();
    Trigger DPadRight = m_operatorController.povRight();
    DPadUp.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.ClimbTop)));
    DPadDown.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.ClimbBottom)));
    DPadLeft.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.Stow)));
    DPadRight.onTrue(new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.Short)));


    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));*/

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_operatorController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand() {
    // An example command will be run in autonomous
   // return Autos.exampleAuto(m_exampleSubsystem);
  }
//}
