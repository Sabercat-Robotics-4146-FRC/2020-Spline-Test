/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Auto.Timer;
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

  private DifferentialDrive diffDrive;

  private Joystick driveController;
  
  private CANSparkMax leftFront;
  private CANSparkMax leftBack;
  private CANSparkMax rightFront;
  private CANSparkMax rightBack;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private CANPIDController leftPidController;
  private CANPIDController rightPidController;
  
  private PigeonIMU pidgey;

  private Timer timer = new Timer();

  private static double accumulator = 0.0;
  private static int counter = 0;

  private boolean isAutoFinished = false; // auto flag for spline
  
  private double totalTime = 5; //seconds
  private double timeStep = 0.02; //period of control loop on motor controller, seconds
  private double robotTrackWidth = 2; //distance between left and right wheels, feet

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    rightFront = new CANSparkMax(1, MotorType.kBrushless);
    rightBack = new CANSparkMax(2, MotorType.kBrushless);
    leftFront = new CANSparkMax(3, MotorType.kBrushless);
    leftBack = new CANSparkMax(4, MotorType.kBrushless);

    rightBack.follow(rightFront);
    leftBack.follow(leftFront);

    rightFront.setSmartCurrentLimit(50);
    rightBack.setSmartCurrentLimit(50);
    leftFront.setSmartCurrentLimit(50);
    leftBack.setSmartCurrentLimit(50);

    rightFront.setClosedLoopRampRate(.1);
    rightBack.setClosedLoopRampRate(.1);
    leftFront.setClosedLoopRampRate(.1);
    leftBack.setClosedLoopRampRate(.1);

    //rightFront.setInverted(true);
    // leftFront.setInverted(true);

    pidgey = new PigeonIMU(5);

    driveController = new Joystick(0);

    leftPidController = leftFront.getPIDController();
    rightPidController = rightFront.getPIDController();

    leftEncoder = leftFront.getEncoder();
    rightEncoder = rightFront.getEncoder();

    leftEncoder.setPosition(0.0);
    rightEncoder.setPosition(0.0);

    diffDrive = new DifferentialDrive(leftFront, rightFront);
    diffDrive.setSafetyEnabled(false);

    pidgey.configFactoryDefault();

    // PID coefficients
    kP = 0.000065;    //0.001;      //0.5;  //5e-5; 
    kI = 0.000000001;   //0.5;  //1e-6;
    kD = 0; 
    kIz = 0; 
    kFF = 0.00017; // ff for velocity
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // left PIDS sets
    leftPidController.setP(kP);
    leftPidController.setI(kI);
    leftPidController.setD(kD);
    leftPidController.setIZone(kIz);
    leftPidController.setFF(kFF);
    leftPidController.setOutputRange(kMinOutput, kMaxOutput);

    // right PID sets
    rightPidController.setP(kP);
    rightPidController.setI(kI);
    rightPidController.setD(kD);
    rightPidController.setIZone(kIz);
    rightPidController.setFF(kFF);
    rightPidController.setOutputRange(kMinOutput, kMaxOutput);
    
    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);

    chooser.setDefaultOption("Default Auto", kDefaultAuto);
    chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", chooser);
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

    SmartDashboard.putNumber("Left Encoder Velocity", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Right Encoder Velocity", rightEncoder.getVelocity());
 
    // SmartDashboard.putNumber("left position", leftEncoder.getPosition());
    // SmartDashboard.putNumber("right Position", rightEncoder.getPosition());

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
    //m_autoSelected = chooser.getSelected();

    timer.reset();

    isAutoFinished = false;

    counter = 0;

    double[][] straightLine = new double[][] {
        {0,0},
        {0,3},
        {0,6}
    };

    // tester arrays
    // double[][] rightTurn = new double[][] {
    //     {0,0},
    //     {0,1},
    //     {0,2},
    //     {1,2},
    //     {2,2}
    // };

    // double[][] leftTurnNeg = new double[][] {
    //   {0,0},
    //   {-1,1}, 
    //   {-2,2}
    // };

    // double[][] leftTurnPos = new double[][] {
    //   {1,1},
    //   {3,1},
    //   {3,3}
    // };

    // double[][] smallCircle = new double[][] {
    //     {0,0.1},
    //     {-2,2},
    //     {0,4},
    //     {2,2},
    //     {0,0}
    // };

    // double[][] bigCircle = new double[][] {
    //     {0,0},
    //     {-4.5,2.5},
    //     {-6,6},
    //     {-4.5,9.5},
    //     {0,12},
    //     {4.5,9.5},
    //     {6,6},
    //     {4.5,2.5},
    //     {0,0}
    // };
    
    final PathPlanner path = new PathPlanner(straightLine);
    path.calculate(totalTime, timeStep, robotTrackWidth);
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

    Robot.accumulator += timer.getDT();
    // System.out.println(timer.getDT());
    
    SmartDashboard.putNumber("left position", leftEncoder.getPosition());
    SmartDashboard.putNumber("right Position", -rightEncoder.getPosition());

    if (!isAutoFinished && (accumulator >= timeStep)) {

      System.out.printf("\n--------------------\nCounter: %d \nLeft Velocity: %f \nRight Velocity: %f", 
        counter, PathPlanner.smoothLeftVelocity[counter][1], PathPlanner.smoothRightVelocity[counter][1]);
    
      leftPidController.setReference(PathPlanner.smoothLeftVelocity[counter][1], ControlType.kVelocity);
      rightPidController.setReference(-PathPlanner.smoothRightVelocity[counter][1], ControlType.kVelocity);

      // test code for pid controllers
      // leftPidController.setReference(720, ControlType.kVelocity);
      // rightPidController.setReference(-720, ControlType.kVelocity); // find way to invert right side to drive forward

      Robot.counter++;
      accumulator = 0.0;

      if (counter == PathPlanner.smoothLeftVelocity.length) {
        isAutoFinished = true;

        leftPidController.setReference(0, ControlType.kVelocity); // set control type to voltage for a harder stop
        rightPidController.setReference(0, ControlType.kVelocity);

      }
    } 
    
    timer.update();
    
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    SmartDashboard.putNumber("left position", leftEncoder.getPosition());
    SmartDashboard.putNumber("right Position", rightEncoder.getPosition());

    diffDrive.arcadeDrive(-driveController.getRawAxis(1), driveController.getRawAxis(4));

    // SmartDashboard.putNumber("Left Enc Position", leftEncoder.getPosition());
    // SmartDashboard.putNumber("Right Enc Position", rightEncoder.getPosition());
  
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
