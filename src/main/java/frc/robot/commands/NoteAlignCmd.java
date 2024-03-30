// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxLimitSwitch;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.IPFSSub;
import frc.robot.subsystems.SwerveSubsystem;

public class NoteAlignCmd extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final IPFSSub m_ipfs;
  private final NetworkTable limelighNetworkTable;
  private final PIDController xPidController, yPidController, rotPidController;
  private double previousPipeline;
  private double tv, tx, ty, ta, lastYaw, xDistance, yDistance;
  private double degreesToRotate;
  private double tOld, tNew;
  private boolean detectNotes, moveToNote, seesNote;

  // Change these values as needed
  private double noteSizeThreshold = 0.3; // Minimum size of the NOTE for the robot to move to and pickup.
  private double overshootDistance = 0.5; // This is how many meters will be added to the distance to try to run over
                                          // the NOTE.
  private double limeLightHeight = 0.71; // The LimeLight distance off the floor in meters, used to estimate distance to
                                         // a NOTE.
  private double limeLightAngle = 0; // Pitch of the LimeLight
  private double minimumNoteAngle = -21.5; // The minimum angle for the LimeLight to calculate distance to the NOTE.
                                           // Below this, the robot will estimate.
  private double translationP = 0.38; // P can be 0.05 after initial testing
  private double translationI = 0;
  private double translationD = 0;

  private double rotationP = 0.0075;
  private double rotationI = 0;
  private double rotationD = 0.0005;
  //

  /**
   * Select the object detection pipeline (on the Pickup LL)
   * Rotate in place until you see a NOTE or notes
   * Center on and drive over the closest note
   * Eventually automatically pick it up
   */

  /** Creates a new DriveAndPickupNoteCmd. */
  public NoteAlignCmd(SwerveSubsystem swerveSubsystem, IPFSSub ipfs) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_ipfs = ipfs;
    this.xPidController = new PIDController(translationP, translationI, translationD);
    this.yPidController = new PIDController(translationP, translationI, translationD);
    this.rotPidController = new PIDController(rotationP, rotationI, rotationD);

    this.limelighNetworkTable = NetworkTableInstance.getDefault()
        .getTable(VisionConstants.kPickupLimelightNetworkTableName);

    xPidController.setSetpoint(0);
    yPidController.setSetpoint(0);
    rotPidController.setSetpoint(0);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limelighNetworkTable.getEntry("pipeline").setNumber(0);
    tv = 0;
    tx = 0;
    ty = 0;
    ta = 0;
    degreesToRotate = 0;
    detectNotes = true; // Try to search for NOTES
    tOld = 0;
    tNew = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Initialize ChassisSpeeds to control robot movement
    ChassisSpeeds chassisSpeeds;
    chassisSpeeds = new ChassisSpeeds(0, 0, 0);

    // If the robot should look for a NOTE, get data from the LimeLight and rotate
    // in place.
    if (detectNotes == true) {
      // Get most recent LimeLight position on a NOTE, if present.
      updateValues();
    }

    updateDistances();
    chassisSpeeds = new ChassisSpeeds(xPidController.calculate(xDistance),
        yPidController.calculate(yDistance + overshootDistance), 0);

    // Convert chassis speeds to individual module states
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    // Output each module states to wheels
    m_swerveSubsystem.setModuleStates(moduleStates);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_ipfs.haveNote()) { // Add or statement for NOTE in pickup
      return true;
    }
    return false;
  }

  private void updateValues() {
    ty = limelighNetworkTable.getEntry("ty").getDouble(0);

    if (ty >= minimumNoteAngle) {
      tv = limelighNetworkTable.getEntry("tv").getDouble(0);
      tx = limelighNetworkTable.getEntry("tx").getDouble(0);
      ty = limelighNetworkTable.getEntry("ty").getDouble(0) + 90 + limeLightAngle;
      ta = limelighNetworkTable.getEntry("ta").getDouble(0);
    } else {
      detectNotes = false;
      tv = 0;
      tx = 0;
      ty = 0;
      ta = 0;
    }
  }

  private void updateDistances() {
    tOld = tNew;
    tNew = System.nanoTime() / Math.pow(10, 9);

    if (detectNotes) {
      yDistance = Math.tan(ty) * limeLightHeight;
      xDistance = Math.tan(tx) * yDistance;
    } else if (tOld != 0 && tNew != 0) {
      yDistance = yDistance - m_swerveSubsystem.getRobotRelativeSpeeds().vyMetersPerSecond * (tNew - tOld);
      xDistance = xDistance - m_swerveSubsystem.getRobotRelativeSpeeds().vxMetersPerSecond * (tNew - tOld);
      SmartDashboard.putNumber("xdistance", xDistance);
      SmartDashboard.putNumber("ydistance", yDistance);
    }
  }
}