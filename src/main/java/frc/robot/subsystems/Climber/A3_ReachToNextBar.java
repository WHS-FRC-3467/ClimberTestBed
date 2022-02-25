// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;

public class A3_ReachToNextBar extends CommandBase {

  ClimberSubsystem m_climber;
  int m_climbPhase = 1;
  Timer m_timer = new Timer();

  public A3_ReachToNextBar(ClimberSubsystem climber) {
    m_climber = climber;
    addRequirements(m_climber);
  }

  @Override
  public void initialize() {
    m_climbPhase = 1;
    m_timer.reset();
  }

  @Override
  public void execute() {

    switch (m_climbPhase) {
      case 1:
        // Adjustable Arms to Angle
        m_climber.extendingClimberAngled();

        m_timer.start();
        SmartDashboard.putString("status", "Phase 1");
        if (m_timer.hasElapsed(3.0)) {
          SmartDashboard.putString("status", "Phase 1 - Complete");
          m_climbPhase = 2;
          m_timer.stop();
          m_timer.reset();
        }
        break;

      case 2:
        // Extend AdjArms to Maximum length
        m_climber.adjustArmsMagically(ClimberConstants.kFullExtendedPosition);

        SmartDashboard.putString("status", "Phase 2");
        if (m_climber.areArmsOnTarget()) {
          SmartDashboard.putString("status", "Phase 2 - On Target");
          m_climbPhase = 3;
        }
        break;

      case 3:
        // Move AdjArms to Vertical position so they rest against next bar
        m_climber.extendingClimberVertical();

        m_timer.start();
        SmartDashboard.putString("status", "Phase 3");
        if (m_timer.hasElapsed(2.0)) {
          SmartDashboard.putString("status", "Phase 3 - Finished");
          m_climbPhase = 0;// Finished
          m_timer.stop();
          m_timer.reset();
        }
        break;

      default:
        SmartDashboard.putString("status", "Phase ???");
        break;
    } 
  
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.adjustArmsManually(0.0);
  }

  @Override
  public boolean isFinished() {
    return (m_climbPhase == 0);
  }

}
