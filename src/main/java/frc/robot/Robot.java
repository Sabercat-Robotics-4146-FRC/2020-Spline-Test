/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import frc.robot.Auto.Timer;
import frc.robot.Subsystems.PIDController;
import frc.robot.Auto.PathPlanner;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private final SendableChooser<String> chooser = new SendableChooser<>();

  private Joystick driveController;
  
  private TalonFX leftFront;
  private TalonFX leftBack;
  private TalonFX rightFront;
  private TalonFX rightBack;

  private TalonFXConfiguration configs;
  
  private PigeonIMU pidgey;

  private int targetAngle;

  private Timer timer = new Timer();

  private static double accumulator = 0.0;
  private static int counter = 0;

  public double[] yprArr;

  private boolean isAutoFinished = false; // auto flag for spline
  
  private double totalTime = 5; //seconds
  private double timeStep = 0.1; //period of control loop on motor controller, seconds
  private double robotTrackWidth = 2; //distance between left and right wheels, feet

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    rightFront = new TalonFX(1);
    rightBack = new TalonFX(2);
    leftFront = new TalonFX(3);
    leftBack = new TalonFX(4);

    rightBack.set(ControlMode.Follower, rightFront.getDeviceID());
    leftBack.set(ControlMode.Follower, leftFront.getDeviceID());
    // rightBack.follow(rightFront);
    // leftBack.follow(leftFront);

    // rightFront.configClosedloopRamp(.1);
    // rightBack.configClosedloopRamp(.1);
    // leftFront.configClosedloopRamp(.1);
    // leftBack.configClosedloopRamp(.1);

    rightFront.setInverted(true);
    rightBack.setInverted(true);
    // leftFront.setInverted(true);

    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    pidgey = new PigeonIMU(17);
    yprArr = new double[]{3};

    // pidgey.enterCalibrationMode(CalibrationMode.Accelerometer);

    driveController = new Joystick(0);

    // encoder values
    configs = new TalonFXConfiguration();
		configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    leftFront.configAllSettings(configs);
    rightFront.configAllSettings(configs);

    pidgey.configFactoryDefault(); 

    rightFront.configRemoteFeedbackFilter(pidgey.getDeviceID(), RemoteSensorSource.Pigeon_Yaw,
      PIDController.REMOTE_1, 30);
      
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, PIDController.PIDTurn, 30);	
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDController.PIDPrimary, 30);
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDController.PIDPrimary, 30);

    rightFront.configAllowableClosedloopError(PIDController.PIDPrimary, 0, 30);
    leftFront.configAllowableClosedloopError(PIDController.PIDPrimary, 0, 30);

    rightFront.selectProfileSlot(PIDController.REMOTE_0, PIDController.PIDPrimary);
    leftFront.selectProfileSlot(PIDController.REMOTE_0, PIDController.PIDPrimary);

    rightFront.selectProfileSlot(PIDController.REMOTE_1, PIDController.PIDTurn);
    leftFront.selectProfileSlot(PIDController.REMOTE_1, PIDController.PIDTurn);

    rightFront.config_kP(0, PIDController.velocity.kP);
    rightFront.config_kI(0, PIDController.velocity.kI);
    rightFront.config_kD(0, PIDController.velocity.kD);
    rightFront.config_kF(0, PIDController.velocity.kF);
    rightFront.config_IntegralZone(0, PIDController.velocity.kIzone);

    leftFront.config_kP(0, PIDController.velocity.kP);
    leftFront.config_kI(0, PIDController.velocity.kI);
    leftFront.config_kD(0, PIDController.velocity.kD);
    leftFront.config_kF(0, PIDController.velocity.kF);
    leftFront.config_IntegralZone(0, PIDController.velocity.kIzone);

    rightFront.config_kP(1, PIDController.turning.kP);
    rightFront.config_kP(1, PIDController.turning.kI);
    rightFront.config_kP(1, PIDController.turning.kD);
    rightFront.config_kP(1, PIDController.turning.kF);
    rightFront.config_IntegralZone(1, PIDController.turning.kIzone);

    // zero pidgey
    pidgey.setYaw(0, 30);
    pidgey.setAccumZAngle(0, 30);
    
    rightFront.setSelectedSensorPosition(0, PIDController.PIDPrimary, 30);
    leftFront.setSelectedSensorPosition(0, PIDController.PIDPrimary, 30);

    //rightFront.configAuxPIDPolarity(false, 30);

    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    SmartDashboard.putNumber("Left Encoder Velocity", leftFront.getSelectedSensorVelocity()); // to get 100 ms to sec
    SmartDashboard.putNumber("Right Encoder Velocity", rightFront.getSelectedSensorVelocity());
    // velocity in position units per 100 ms
        
    SmartDashboard.putNumber("left position", leftFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("right Position", rightFront.getSelectedSensorPosition());
    // position units

    SmartDashboard.putNumber("%out", driveController.getY());
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    targetAngle = rightFront.getSelectedSensorPosition(1);

    // resets counter to 0 w/o reuploading code
    counter = 0;

    timer.reset();

    isAutoFinished = false;

    double[][] straightLine = new double[][] {
      {0,0},
      {0,3},
      {0,6}
        // {3,0},
        // {3,0.25},
        // {6,8},
        // {2,15},
        // {2,20}
    };

    // tester arrays
    // double[][] rightTurn = new double[][] {
    //     {0,0},
    //     {0,1},
    //     {0,2},
    //     {1,2},
    //     {2,2}
    // };

    // double[][] leftTurnPos = new double[][] {
    //   {1,1},
    //   {3,1},
    //   {3,3}
    // };

    
    final PathPlanner path = new PathPlanner(straightLine);
    path.calculate(totalTime, timeStep, robotTrackWidth);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    // pidgey.getYawPitchRoll(yprArr);
    // System.out.print("yaw: " + yprArr[0] + " ");
    System.out.println("pos: " + targetAngle);

    accumulator += timer.getDT();
    SmartDashboard.putNumber("acum", accumulator);
    // System.out.println(accumulator);


    if (!isAutoFinished && (accumulator >= PathPlanner.smoothRightVelocity[counter][0])) {

      // System.out.printf("\n--------------------\nCounter: %d \nLeft Velocity: %f \nRight Velocity: %f", 
      //   counter, PathPlanner.smoothLeftVelocity[counter][1], PathPlanner.smoothRightVelocity[counter][1]);

      rightFront.set(ControlMode.Velocity, PathPlanner.smoothRightVelocity[counter][1], DemandType.AuxPID, targetAngle);
      leftFront.set(ControlMode.Velocity, PathPlanner.smoothRightVelocity[counter][1], DemandType.AuxPID, targetAngle);

      // rightFront.set(ControlMode.Velocity, 4000, DemandType.AuxPID, targetAngle);
      // leftFront.set(ControlMode.Velocity, 4000, DemandType.AuxPID, targetAngle);

      // rightFront.set(ControlMode.Velocity, 4000);
      // leftFront.set(ControlMode.Velocity, 4000);

      // SmartDashboard.putNumber("right velocity", PathPlanner.smoothRightVelocity[counter][1]);
      // SmartDashboard.putNumber("left velocity", PathPlanner.smoothLeftVelocity[counter][1]);

      counter++;
      // accumulator = 0.0;

      if (counter >= PathPlanner.smoothLeftVelocity.length) {
        isAutoFinished = true;

        rightFront.set(ControlMode.Velocity, 0);  // voltage for a harder stop
        leftFront.set(ControlMode.Velocity, 0);

      }
    } 
    
    timer.update();
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    double forward = -1 * driveController.getY();
    double turn = driveController.getRawAxis(4);
    forward = Deadband(forward);
    turn = Deadband(turn);
    
    // leftFront.set(ControlMode.Velocity, forward, DemandType.ArbitraryFeedForward, turn);
    // rightFront.set(ControlMode.Velocity, forward, DemandType.ArbitraryFeedForward, -turn);
    leftFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
		rightFront.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
  
  }

  double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}