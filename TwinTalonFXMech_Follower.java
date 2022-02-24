// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.DemandType;
//import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ClimberConstants;
import frc.robot.sim.PhysicsSim;

/**
 *  This is Motion Magic with the Auxiliary PID using the difference
 *  between two encoders to maintain a straight heading.
 * /

/** Add your docs here. */
public class TwinTalonFXMech implements Sendable {

    /* Enable SmartDash Output? */
	boolean m_debugging = true;	
	int debug_counter = 0;	
		
    /** Hardware */
	WPI_TalonFX m_leftFollower;
	WPI_TalonFX m_rightMaster;
	
	/** Invert Directions for Left and Right */
	TalonFXInvertType m_leftInvert = TalonFXInvertType.CounterClockwise; //Same as invert = "false"
	TalonFXInvertType m_rightInvert = TalonFXInvertType.Clockwise; //Same as invert = "true"

	/** Config Objects for motor controllers */
	TalonFXConfiguration m_leftConfig = new TalonFXConfiguration();
	TalonFXConfiguration m_rightConfig = new TalonFXConfiguration();
	
    /** Default setpoint */
    private double m_setpoint = 0.0;

    public TwinTalonFXMech (int leftTalonID, int rightTalonID) {

		/* Instantiate Hardware */
        m_leftFollower = new WPI_TalonFX(leftTalonID, "rio");
        m_rightMaster = new WPI_TalonFX(rightTalonID, "rio");
        
		/* Disable all motor controllers */
		m_rightMaster.set(TalonFXControlMode.PercentOutput, 0);
		m_leftFollower.set(TalonFXControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		m_leftFollower.setNeutralMode(NeutralMode.Brake);
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local Integrated Sensor */
		m_leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();	// Local Feedback Source

		/* Configure the Remote (left) Talon's selected sensor as a remote sensor for the right Talon */
	//	m_rightConfig.remoteFilter0.remoteSensorDeviceID = m_leftFollower.getDeviceID(); // Device ID of Source
	//	m_rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; // Remote Feedback Source
		m_rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

		/* Now that the Left sensor can be used by the master Talon,
		 * set up the Left (Aux) and Right (Master) distance into a single
		 * Robot distance as the Master's Selected Sensor 0. */
	//	m_rightConfig.diff0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
	//	m_rightConfig.diff1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
	//	m_rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Diff0 - Diff1
	//	m_rightConfig.primaryPID.selectedFeedbackCoefficient = 0.5;
	
		/* Setup difference signal to be used for turn when performing Drive Straight with encoders */
	//	m_rightConfig.sum0Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Local Integrated Sensor
	//	m_rightConfig.sum1Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();   //Aux Selected Sensor
	//	m_rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1
	//	m_rightConfig.auxPIDPolarity = true;
	//	m_rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 1.0;

		/* Configure neutral deadband */
		m_rightConfig.neutralDeadband = ClimberConstants.kNeutralDeadband;
		m_leftConfig.neutralDeadband = ClimberConstants.kNeutralDeadband;
		
		/* Motion Magic Configurations */
		m_rightConfig.motionAcceleration = ClimberConstants.kMotionAcceleration;
		m_rightConfig.motionCruiseVelocity = ClimberConstants.kMotionCruiseVelocity;

		/**
		 * Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		m_leftConfig.peakOutputForward = +1.0;
		m_leftConfig.peakOutputReverse = -1.0;
		m_rightConfig.peakOutputForward = +1.0;
		m_rightConfig.peakOutputReverse = -1.0;

		/* FPID Gains for distance servo */
		m_rightConfig.slot0.kP = ClimberConstants.kGains_Distance.kP;
		m_rightConfig.slot0.kI = ClimberConstants.kGains_Distance.kI;
		m_rightConfig.slot0.kD = ClimberConstants.kGains_Distance.kD;
		m_rightConfig.slot0.kF = ClimberConstants.kGains_Distance.kF;
		m_rightConfig.slot0.integralZone = ClimberConstants.kGains_Distance.kIzone;
		m_rightConfig.slot0.closedLoopPeakOutput = ClimberConstants.kGains_Distance.kPeakOutput;
		m_rightConfig.slot0.allowableClosedloopError = 0;

		/* FPID Gains for turn servo */
	//	m_rightConfig.slot1.kP = ClimberConstants.kGains_Turning.kP;
	//	m_rightConfig.slot1.kI = ClimberConstants.kGains_Turning.kI;
	//	m_rightConfig.slot1.kD = ClimberConstants.kGains_Turning.kD;
	//	m_rightConfig.slot1.kF = ClimberConstants.kGains_Turning.kF;
	//	m_rightConfig.slot1.integralZone = ClimberConstants.kGains_Turning.kIzone;
	//	m_rightConfig.slot1.closedLoopPeakOutput = ClimberConstants.kGains_Turning.kPeakOutput;
	//	m_rightConfig.slot1.allowableClosedloopError = 0;

		/**
		 * 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		m_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		//m_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;

		m_leftFollower.configAllSettings(m_leftConfig);
		m_rightMaster.configAllSettings(m_rightConfig);
		
		/* Configure output and sensor direction */
		m_leftFollower.setInverted(m_leftInvert);
		m_rightMaster.setInverted(m_rightInvert);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // m_leftFollower.setSensorPhase(true);
        // m_rightMaster.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 10);
		m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 10);

		//m_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 10, 10);
		//m_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, 10);

		//m_leftFollower.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 10);

        /* Configure Smoothing */
        m_rightMaster.configMotionSCurveStrength(ClimberConstants.kCurveSmoothing);

		/* Determine which slot affects which PID */
		m_rightMaster.selectProfileSlot(0, 0);
		m_leftFollower.selectProfileSlot(0, 0);
		//m_rightMaster.selectProfileSlot(1, 1);

        /* Initialize sensors */
		zeroSensors();

		/* Set up the Talons in Simulation */
		if (Robot.isSimulation()) {
			// simulationInit
			PhysicsSim.getInstance().addTalonFX(m_leftFollower, 0.5, 6800, false);
			PhysicsSim.getInstance().addTalonFX(m_rightMaster, 0.5, 6800, false);
		}
	}
	
	/*
     * Manual control
     */
	public void drive(double speed) {
		m_rightMaster.set(ControlMode.PercentOutput, speed);
		m_leftFollower.set(ControlMode.PercentOutput, speed);

		reportMotionToDashboard();
	}


	public double calibrate(boolean left) {

		double current;
		
		// Drive arms slowly in
		if (left) {
			m_rightMaster.set(ControlMode.PercentOutput, -0.30);
			current = m_leftFollower.getStatorCurrent();
			SmartDashboard.putNumber("Calib Curr L", current);
		} else {
			m_leftFollower.set(ControlMode.PercentOutput, -0.30);
			current = m_rightMaster.getStatorCurrent();
			SmartDashboard.putNumber("Calib Curr R", current);
		}
		
		return current;
	}

	public void runMotionMagic() {
		
		// Run Motion Magic using current value of m_setPoint as target

		/* Left Talon will follow the Right Talon */
        //m_leftFollower.follow(m_rightMaster, FollowerType.AuxOutput1);
        m_leftFollower.follow(m_rightMaster, FollowerType.PercentOutput);

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
        /* Keeping the target setpoint for the Difference PID at 0 will keep the two Talons tracking "straight" */ 
		//m_rightMaster.set(TalonFXControlMode.MotionMagic, m_setpoint, DemandType.AuxPID, 0);
		m_rightMaster.set(TalonFXControlMode.MotionMagic, m_setpoint);

		reportMotionToDashboard();
	}
	
	public void runMotionMagic(double setpoint) {
	
		/* Left Talon will follow the Right Talon */
        //m_leftFollower.follow(m_rightMaster, FollowerType.AuxOutput1);
        m_leftFollower.follow(m_rightMaster, FollowerType.PercentOutput);

		/* Configured for MotionMagic on Integrated Sensors' Sum and Auxiliary PID on Integrated Sensors' Difference */
        /* Keeping the target setpoint for the Difference PID at 0 will keep the two Talons tracking "straight" */ 
		//m_rightMaster.set(TalonFXControlMode.MotionMagic, setpoint, DemandType.AuxPID, 0);
		m_rightMaster.set(TalonFXControlMode.MotionMagic, setpoint);

    	m_setpoint = setpoint;

		reportMotionToDashboard();
	}
	
	// Loop counter for determining stability around target setpoint
	int m_withinThresholdLoops = 0;
	
	public boolean isMechOnTarget() {
	
		final int kErrThreshold = 10; // how many sensor units until it's close-enough?
		final int kLoopsToSettle = 10; // # of loops for which the sensor must be close-enough
	
		// Get current target and determine how far we are from it (error)
		int target = (int) m_rightMaster.getClosedLoopTarget();
		int error = (int) (target - m_rightMaster.getActiveTrajectoryPosition()); // Use this for Motion Magic
	
		/* Check if closed loop error is within the threshld */
		if (Math.abs(error) <= kErrThreshold) {
			++m_withinThresholdLoops;
		} else {
			m_withinThresholdLoops = 0;
		}
		return (m_withinThresholdLoops > kLoopsToSettle);
	}

	public void reportMotionToDashboard() {

		// These are things that we cannot change on SDB; just report their current values
		if (m_debugging && ++debug_counter > 10) {
			SmartDashboard.putString("Arms ControlMode", getTalonControlMode());
	    	SmartDashboard.putNumber("Arms Sensor Position", m_rightMaster.getSelectedSensorPosition(0));
			SmartDashboard.putNumber("Arms MotorOutputPercent", m_rightMaster.getMotorOutputPercent());
			SmartDashboard.putNumber("Arms Current Draw", m_rightMaster.getStatorCurrent());
	    	
			if (m_rightMaster.getControlMode() == ControlMode.MotionMagic) {
				SmartDashboard.putNumber("Arms Traj. Position", m_rightMaster.getActiveTrajectoryPosition());
				SmartDashboard.putNumber("Arms ClosedLoopTarget", m_rightMaster.getClosedLoopTarget(0));
				SmartDashboard.putNumber("Arms ClosedLoopError", m_rightMaster.getClosedLoopError(0));
			}
			debug_counter = 0;
		}
	}
	
	/**
	 * @return The current TalonSRX control mode
	 */
	public String getTalonControlMode() {
		
		ControlMode tcm = m_rightMaster.getControlMode();
		
		if (tcm == ControlMode.PercentOutput) {
			return "PercentOutput";
		}
		else if (tcm == ControlMode.MotionMagic) {
			return "MotionMagic";
		}
		else
			return "Other";
	}

	/** Zero integrated encoders on Talons */
	void zeroSensors() {
		m_leftFollower.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
		m_rightMaster.getSensorCollection().setIntegratedSensorPosition(0, ClimberConstants.kTimeoutMs);
		System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
		reportMotionToDashboard();
	}
	
    /** Configure Curve Smoothing */
    void configureSmoothing(int smooth) {
        m_rightMaster.configMotionSCurveStrength(smooth);
    }

    /** Return Talon Objects */
    TalonFX getLeftTalon() {
        return (TalonFX)m_leftFollower;
    }

    TalonFX getRightTalon() {
        return (TalonFX)m_rightMaster;
    }

	private void setSetpoint(double sp)  { m_setpoint = sp; }
	private double getSetpoint()  { return m_setpoint; }
    
	private void setP(double p)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kP(0, p); }
	private double getP()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_P, 0); }

	private void setI(double i)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kI(0, i); }
	private double getI()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_I, 0); }

	private void setD(double d)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kD(0, d); }
	private double getD()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_D, 0); }

	private void setF(double f)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.config_kF(0, f, 0); }
	private double getF()  { return m_rightMaster.configGetParameter(ParamEnum.eProfileParamSlot_F, 0, 0); }

	private void setMMAccel(double acc)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.configMotionAcceleration(acc); }
	private double getMMAccel()  { return m_rightMaster.configGetParameter(ParamEnum.eMotMag_Accel, 0, 0); }

	private void setMMCruise(double cru)  {m_rightMaster.selectProfileSlot(0, 0); m_rightMaster.configMotionCruiseVelocity(cru); }
	private double getMMCruise()  { return m_rightMaster.configGetParameter(ParamEnum.eMotMag_VelCruise, 0, 0); }

	@Override
	public void initSendable(SendableBuilder builder) {
	  builder.setSmartDashboardType("Robot Preferences");
	  builder.addDoubleProperty("P", this::getP, this::setP);
	  builder.addDoubleProperty("I", this::getI, this::setI);
	  builder.addDoubleProperty("D", this::getD, this::setD);
	  builder.addDoubleProperty("F", this::getF, this::setF);
	  builder.addDoubleProperty("Setpoint", this::getSetpoint, this::setSetpoint);
	  builder.addDoubleProperty("Accel", this::getMMAccel, this::setMMAccel);
	  builder.addDoubleProperty("Cruise", this::getMMCruise, this::setMMCruise);
	}

}
