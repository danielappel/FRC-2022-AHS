// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

//import com.ctre.phoenix.motorcontrol.*;
//import com.ctre.phoenix.motorcontrol.can.*;



/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  // Motor Declarations
  PWMVictorSPX motor_rightRear = new PWMVictorSPX(0);
  PWMVictorSPX motor_rightFront = new PWMVictorSPX(1);
  PWMVictorSPX motor_leftRear = new PWMVictorSPX(3);
  PWMVictorSPX motor_leftFront = new PWMVictorSPX(2);

  /**
   * private final PWMSparkMax m_leftMotor = new PWMSparkMax(0);
  private final PWMSparkMax m_rightMotor = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);
  private final Joystick m_stick = new Joystick(0);

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
  private final Joystick m_stick = new Joystick(0);
  private final Joystick driverTwoJoystick = new Joystick(1);


  

 

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
     mRight.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-m_stick.getY(), m_stick.getX());

    if(m_stick.getRawButtonPressed(1)) {
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

  public void conveyerBelt(){
    if (driverTwoJoystick.getRawButtonPressed(1)){
      
    }

  }

}
