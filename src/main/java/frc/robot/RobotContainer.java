// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.LEDConstants.ledMode;
import frc.robot.Constants.LiftConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IPFSSub;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.AprilTagAlignCmd;
import frc.robot.commands.FeedandFireSpeak;
import frc.robot.commands.ManualLift;
import frc.robot.commands.NoteAlignCmd;
import frc.robot.commands.Pickup;
import frc.robot.commands.SetHeight;
import frc.robot.commands.SwerveJoystickCmd;



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
   private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        private final SendableChooser<Command> autoChooser;


        protected final static SendableChooser<ledMode> LED_Chooser=new SendableChooser<>();


        public final LEDs m_LEDs;

        private final Joystick m_driveController = new Joystick(OIConstants.kDriverControllerPort);
        private final CommandXboxController m_operatorController =
         new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_IPFSSub = new IPFSSub();
    m_Lift = new Lift();
    configureBindings();
    //m_Lift.setDefaultCommand(new SetHeight(m_Lift, LiftConstants.defaultStartingHeight));
    m_Lift.setDefaultCommand(new ManualLift(m_Lift));


    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> m_driveController.getRawAxis(OIConstants.kDriverYAxis),
                                () -> m_driveController.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -m_driveController.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> !m_driveController.getRawButton(5))); // LB

                 //Register named commands
              NamedCommands.registerCommand("NoteAlignedCmd", new NoteAlignCmd(swerveSubsystem));
              NamedCommands.registerCommand("AprilTagAlignCmd", new AprilTagAlignCmd(swerveSubsystem));

              // Build an auto chooser. This will use Commands.none() as the default option.
              autoChooser = AutoBuilder.buildAutoChooser();

                // Another option that allows you to specify the default auto by its name
                // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

                SmartDashboard.putData("Auto Chooser", autoChooser);


                LED_Chooser.setDefaultOption("RED", ledMode.RED);
                LED_Chooser.addOption("BLUE", ledMode.BLUE);
                LED_Chooser.addOption("PURPLE", ledMode.PURPLE);
                LED_Chooser.addOption("GREEN", ledMode.GREEN);
                LED_Chooser.addOption("RAINBOW", ledMode.RAINBOW);
                LED_Chooser.addOption("YELLOW", ledMode.YELLOW);
                LED_Chooser.addOption("TEAM", ledMode.TEAM);
                LED_Chooser.addOption("ALLIANCE", ledMode.ALLIANCE);
          
                SmartDashboard.putData("LED COLORS", LED_Chooser);

                m_LEDs = new LEDs();
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
    //LB = Short
    //RB = Stow
    //uDPad = Pickup
    //dDPad = Climb Back
    //lDPad = Climb Left
    //rDPad = Climb Right
    //A = Amp
    //X = Hi
    //Y = Mid
    //B = Low
    Trigger OPaButton = m_operatorController.a();
    Trigger OPbButton = m_operatorController.b();
    Trigger OPyButton = m_operatorController.y();
    Trigger OPxButton = m_operatorController.x();
    Trigger OPlBumper = m_operatorController.leftBumper();
    Trigger OPrBumper = m_operatorController.rightBumper();

    OPrBumper.onTrue(new FeedandFireSpeak(m_IPFSSub));
    OPlBumper.whileTrue(new Pickup(m_IPFSSub));



    Trigger OPuDPad = m_operatorController.povUp();
    Trigger OPdDPad = m_operatorController.povDown();
    Trigger OPlDPad = m_operatorController.povLeft();
    //Trigger OPrDPad = m_operatorController.povRight();
    //OPuDPad.onTrue(new ParallelDeadlineGroup(new Pickup(m_IPFSSub), new InstantCommand(() -> m_Lift.setLiftSetpoint(LiftConstants.PickupHeight))));
    
    // Press and hold the B button to Pathfind to Roughly Source. Releasing button should cancel the command
     OPdDPad.whileTrue(AutoBuilder.pathfindToPose(
      new Pose2d(15.75, 1.73, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        3.0, 1.0, 
        Units.degreesToRadians(180), Units.degreesToRadians(270)
      ), 
      0, 
      2.0
    ));

  // Press and hold the Y button to Pathfind to (1.83, 3.0, 0 degrees). Releasing button should cancel the command
  OPlDPad.whileTrue(AutoBuilder.pathfindToPose(
      new Pose2d(2.88, 6.99, Rotation2d.fromDegrees(0)), 
      new PathConstraints(
        1.0, 1.0, 
        Units.degreesToRadians(180), Units.degreesToRadians(270)
      ), 
      0, 
      2.0
    ));

  //Press and hold the X button to Pathfind to the start of the "AMP-Path" path. Releasing the button should cancel the command
  OPaButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("AMP-path"), 
      new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
      0.0));

  OPxButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("SpHigh"), 
      new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
      0.0));

  OPyButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("SpMid"), 
      new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
      0.0));

  OPbButton.whileTrue(AutoBuilder.pathfindThenFollowPath(
      PathPlannerPath.fromPathFile("SpLow"), 
      new PathConstraints(1.0, 1.0, Units.degreesToRadians(180), Units.degreesToRadians(270)), 
      0.0));
      

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
