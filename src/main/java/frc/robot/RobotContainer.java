// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber.A0_CalibrateClimber;
import frc.robot.subsystems.Climber.A1_PrepareToClimb;
import frc.robot.subsystems.Climber.A2_LiftToBar;
import frc.robot.subsystems.Climber.A3_ReachToNextBar;
import frc.robot.subsystems.Climber.A4_HookToNextBar;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Climber.MagicClimbByDash;
import frc.robot.subsystems.Climber.MagicClimbByStick;
import frc.robot.subsystems.Climber.ManualClimbByStick;
//import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Control.XboxControllerDPad;
import frc.robot.Control.XboxControllerButton;
import frc.robot.Control.XboxControllerEE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final XboxControllerEE m_operatorController = new XboxControllerEE(1);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    new Pneumactics();
  
    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(DriveConstants.PRACTICE);

    // Configure the button bindings
    configureButtonBindings();

    m_climberSubsystem.setDefaultCommand(new ManualClimbByStick(m_climberSubsystem, 
                                        () -> m_operatorController.getRightY()));

    // Make the Climb Sequence commands available on SmartDash
    SmartDashboard.putData(new A0_CalibrateClimber(m_climberSubsystem));
    SmartDashboard.putData(new A1_PrepareToClimb(m_climberSubsystem /*, m_intakeSubsystem */));
    SmartDashboard.putData(new A2_LiftToBar(m_climberSubsystem));
    SmartDashboard.putData(new A3_ReachToNextBar(m_climberSubsystem));
    SmartDashboard.putData(new A4_HookToNextBar(m_climberSubsystem));
    
    // Arm Driving Commands
    SmartDashboard.putData(new ManualClimbByStick(m_climberSubsystem, () -> m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByStick(m_climberSubsystem, () -> m_operatorController.getRightY()));
    SmartDashboard.putData(new MagicClimbByDash(m_climberSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new XboxControllerButton(m_operatorController, XboxControllerEE.Button.kA)
    .whenPressed(new InstantCommand(m_climberSubsystem::zeroSensors));
        
    new XboxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadUp)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberVertical));
    
    new XboxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadDown)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::fixedClimberAngled));

    new XboxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadLeft)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberAngled));
    
    new XboxControllerDPad(m_operatorController, XboxControllerEE.DPad.kDPadRight)
    .whileActiveContinuous(new InstantCommand(m_climberSubsystem::extendingClimberVertical));
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
