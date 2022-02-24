// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumactics extends SubsystemBase{

  private Compressor m_Compressor;
  private int counter = 0;

  // Pneumactics class constructor
  public Pneumactics() {

    m_Compressor = new Compressor(Constants.PHConstants.PMType);
    if (Constants.PHConstants.UseREVPH) {
      m_Compressor.enableAnalog(120, 130);
    } else {
      m_Compressor.enableDigital();
    }
  }
  
  // This method will be called once per scheduler run
  public void periodic() {

    if (++ counter > 200) {
      SmartDashboard.putBoolean("Compressor On", m_Compressor.enabled());
      SmartDashboard.putBoolean("Pressure Low", m_Compressor.getPressureSwitchValue());
      SmartDashboard.putNumber("Pressure (PSI)", m_Compressor.getPressure());
      SmartDashboard.putNumber("Compressor Current (Amps)", m_Compressor.getCurrent());
      counter = 0;
    }
  }
}
