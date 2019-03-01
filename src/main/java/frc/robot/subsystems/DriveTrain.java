/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.OperatorDrive;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  final WPI_TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
  final DifferentialDrive mDiffDrive;

  public DriveTrain(int port1, int port2, int port3, int port4) {
    this.leftMaster = new WPI_TalonSRX(port1);
    this.leftSlave = new WPI_TalonSRX(port2);
    this.rightMaster = new WPI_TalonSRX(port3);
    this.rightSlave = new WPI_TalonSRX(port4);
    
    leftSlave.set(ControlMode.Follower, leftMaster.getDeviceID());
    rightSlave.set(ControlMode.Follower, rightMaster.getDeviceID());

    mDiffDrive = new DifferentialDrive(leftMaster, rightMaster);
  }

  public DifferentialDrive getDrive() {
    return mDiffDrive;
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new OperatorDrive());
  }
}
