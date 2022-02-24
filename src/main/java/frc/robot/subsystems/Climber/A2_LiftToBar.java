// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class A2_LiftToBar extends CommandBase {

  ClimberSubsystem m_climber;
  int m_climbPhase = 1;

  public A2_LiftToBar(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climbPhase = 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // Adjustable Arms to Vertical
    m_climber.extendingClimberVertical();

    switch (m_climbPhase) {
      case 1:
        // Retract Arms to Minimum Length
        m_climber.adjustArmsMagically(ClimberConstants.kRetractedPostion);
        SmartDashboard.putString("status", "Phase 1");
        if (m_climber.areArmsOnTarget()) {
          SmartDashboard.putString("status", "Phase 1 - On Target");
          m_climbPhase = 2;
        }
        break;

      case 2:
        // Extend arms above the bar so the fixed hooks settle onto bar  
        m_climber.adjustArmsMagically(ClimberConstants.kExtendedAboveBar);
        SmartDashboard.putString("status", "Phase 2");
        if (m_climber.areArmsOnTarget()) {
          SmartDashboard.putString("status", "Phase 2 - Finished");
          m_climbPhase = 0;  // Finished
        }
      break;

      default:
        SmartDashboard.putString("status", "Phase ???");
        break;
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.adjustArmsManually(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_climbPhase == 0);
  }
}
