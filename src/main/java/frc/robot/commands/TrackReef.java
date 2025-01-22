// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackReef extends Command {
  /** Creates a new TrackReef. */
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem;
  private final SwerveSubsystem m_SwerveSubsystem;

  private double xPidOutput;
  private double yPidOutput;
  private double rotationOutput;

  public TrackReef(PhotonVisionSubsystem photonVisionSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_PhotonVisionSubsystem = photonVisionSubsystem;
    this.m_SwerveSubsystem = swerveSubsystem;

    addRequirements(m_PhotonVisionSubsystem, m_SwerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  //   if(m_PhotonVisionSubsystem.getRotationPidError() >= 5) {
  //     m_PhotonVisionSubsystem.getYPidOutput();
  // }
  //   if(m_PhotonVisionSubsystem.getYPidError() >= 5) {
  //     m_PhotonVisionSubsystem.getYPidOutput();
  // }
  //   if(m_PhotonVisionSubsystem.getXPidError() >= 5) {
  //     m_PhotonVisionSubsystem.getXPidOutput();
  // }

    m_SwerveSubsystem.drive(yPidOutput, xPidOutput, rotationOutput, false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}