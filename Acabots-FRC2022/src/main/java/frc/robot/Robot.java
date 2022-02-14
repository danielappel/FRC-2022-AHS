// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.wpilibj.Servo;

//Limelight Imports
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.*;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // Drive Motor Declarations
  PWMVictorSPX motor_rightRear = new PWMVictorSPX(0);
  PWMVictorSPX motor_rightFront = new PWMVictorSPX(1);
  PWMVictorSPX motor_leftFront = new PWMVictorSPX(2);
  PWMVictorSPX motor_leftRear = new PWMVictorSPX(3);

  //Motor that spins intake wheels
  PWMVictorSPX intakeWheels = new PWMVictorSPX(4);

  //Motor that rotates intake
  PWMVictorSPX intakeArm = new PWMVictorSPX(5);

  //Motors that spin conveyor belts (raise and lower balls)
  PWMSparkMax conveyor1 = new PWMSparkMax(6);
  PWMSparkMax conveyor2 = new PWMSparkMax(7);
  //Flywheel motors
  PWMSparkMax shootBallLeft = new PWMSparkMax(8);
  PWMSparkMax shootBallRight = new PWMSparkMax(9);



  /**
   * private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick mainDriverStick = new Joystick(0);

  Spark m_frontLeft = new Spark(1);
  Spark m_rearLeft = new Spark(2);
  MotorControllerGroup m_left = new MotorControllerGroup(m_frontLeft, m_rearLeft);
   * 
   */
  // Speed Declarations
  MotorControllerGroup mRight = new MotorControllerGroup(motor_rightRear, motor_rightFront);
  MotorControllerGroup mLeft = new MotorControllerGroup(motor_leftRear, motor_leftFront);

// Differential Drive Declaration
  DifferentialDrive m_robotDrive = new DifferentialDrive(mLeft, mRight);
  private final Joystick mainDriverStick = new Joystick(0);

  //Xbox controller??
  private final Joystick driverTwoJoystick = new Joystick(1);

  //Modify so that twist to turn isn't so severe.  Adjust as needed.
  private final double TWIST_MULTIPLIER = 0.5;
  private final double THROTTLE_MULTIPLIER  = 0.75;
  private final double ARM_SPEED = 0.25;
  private final double INTAKE_WHEELS_SPEED = 0.5;


//Flywheel shooter speed
  private double shootSpeed = 0;

  Servo cameraServo = new Servo (10);


  

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
     mRight.setInverted(true);
     cameraControls();
     cameraGet();
  }

  public void robotCamera(){
   //UsbCamera usbCamera = new UsbCamera("USB Camera 0", 0);
/*
   camera.setResolution(640, 480);
   CvSink cvSink = CameraServer.getVideo();
   CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);*/
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.

    /*intake motor
    right Xbox joystick - only x-axis movement
    TO DO: adjust by a multiplier to maybe make less severe
    */

    mainDriverControls();

    secondDriverControls();

    cameraControls();


    //testButtons();

  }

  public void mainDriverControls(){
    //General driver controls
    m_robotDrive.arcadeDrive(-THROTTLE_MULTIPLIER*mainDriverStick.getY(), TWIST_MULTIPLIER*mainDriverStick.getTwist());

    //Flywheel shooter function
   
    shooter();

  }

  public void secondDriverControls(){

    intakeRotate();
    conveyor();
  }

  public void cameraGet(){
    //if (driverTwoJoystick.getRawButtonPressed(2))
    {
      UsbCamera camera = CameraServer.startAutomaticCapture();
      //CvSink cvSink = CameraServer.getVideo();
      //CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);
     
      
    }
  }
  public void testButtons()
  {
    //System.out.print("x: " + mainDriverStick.getRawAxis(5));
    //System.out.print("y: " + mainDriverStick.getRawAxis(6));

    if(driverTwoJoystick.getRawAxis(0) !=0) {
      System.out.println("Driver Two Stick Axis 0");
    }
    
    if(mainDriverStick.getRawButtonPressed(1)) {
      System.out.println("This is button 1");

    }

    if(driverTwoJoystick.getRawButtonPressed(1)) {
      System.out.println("Driver two button 1");
    }

    if(driverTwoJoystick.getRawButtonPressed(2)){
      System.out.println("Driver two button 2");
    }

    if (driverTwoJoystick.getRawButtonPressed(3)){
      System.out.println("Driver two button 3");
    }

    if (driverTwoJoystick.getRawButtonPressed(4)){
      System.out.println("Driver two button 4");
    }

    if (driverTwoJoystick.getRawButtonPressed(5)){
      System.out.println("Driver two button 5");
    }

    if (driverTwoJoystick.getRawButtonPressed(6)){
      System.out.println("Driver two button 6");
    }

    

  }

  public  void cameraControls(){
   // cameraServo.setAngle(0);
    if (mainDriverStick.getRawButtonPressed(3)){
      System.out.println("Camera Motor running Up");
      cameraServo.setAngle(cameraServo.getAngle()+6);
    }
    if (mainDriverStick.getRawButtonPressed(4)){
      System.out.println("Camera Motor Running Down");
      cameraServo.setAngle(cameraServo.getAngle()-6);
    }
  }

  public void shooter(){

    if(mainDriverStick.getRawButtonPressed(1)){
      System.out.println("Trigger pulled");
      //Start to spin flywheel at max speed
      shootSpeed = 1;

    }

    if(mainDriverStick.getRawButtonReleased(1)) {
      //Turn off flywheel
      shootSpeed = 0;
    }

    //Continually updated shoot speed;
    shootBallLeft.set(-1*shootSpeed);
    shootBallRight.set(shootSpeed);
  }

  public void intakeRotate(){
      if (driverTwoJoystick.getRawButtonPressed(4)){
        System.out.println("Arm moving in");
        intakeArm.set(ARM_SPEED);
        intakeWheels.set(INTAKE_WHEELS_SPEED); // spin wheels if bringing arm up
      }
      else if (driverTwoJoystick.getRawButtonPressed(2)){
        System.out.println("Arm moving out"); 
        intakeArm.set(-ARM_SPEED);
        intakeWheels.set(0); //deactivate wheels if bring arm back down
      }
      else if (driverTwoJoystick.getRawButtonReleased(4) || driverTwoJoystick.getRawButtonReleased(2))
      {
        intakeArm.set(0);
        intakeWheels.set(0);
      }
    }

  public void conveyor()
  {
      conveyor1.set(driverTwoJoystick.getRawAxis(0));
      conveyor2.set(-driverTwoJoystick.getRawAxis(0));
  }
}


