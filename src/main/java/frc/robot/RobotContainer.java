// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Climber.ClimberSubsystem;
import frc.robot.Climber.ExtendClimber;
import frc.robot.Control.XBoxControllerDPad;
import frc.robot.Control.XboxControllerEE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final XboxControllerEE m_operatorController = new XboxControllerEE(1);
  Compressor pcmCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_climberSubsystem.setDefaultCommand(new ExtendClimber(m_climberSubsystem, 
                                        () -> m_operatorController.getRightY()));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadUp)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberForward, m_climberSubsystem));
    
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadDown)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberReverse, m_climberSubsystem));

    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadLeft)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberForward, m_climberSubsystem));
    
    new XBoxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadRight)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberReverse, m_climberSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
