// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualClimbByStick extends CommandBase {

  ClimberSubsystem m_climber;
  DoubleSupplier m_speed;

  public ManualClimbByStick(ClimberSubsystem climber, DoubleSupplier speed) {
    m_speed = speed;
    m_climber = climber;
    addRequirements(m_climber);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.adjustArmsManually((-1.0) * modifyAxis(m_speed.getAsDouble()));
  }

// Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.adjustArmsManually(0.0);
  }

// Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

	private static double modifyAxis(double value) {
      
		// Apply Deadband
		  double dband = 0.2;
		  if (Math.abs(value) > dband) {
			if (value > 0.0) {
				return (value - dband) / (1.0 - dband);
			} else {
				return (value + dband) / (1.0 - dband);
			}
		  } else {
			value = 0.0;
		  }
		
		// Square the axis for better control range at low speeds
		value = Math.copySign(value * value, value);
	
		return value;
	  }

}
