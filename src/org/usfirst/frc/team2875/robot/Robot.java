/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2875.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.AnalogGyro;

public class Robot extends IterativeRobot { 

	public static final double[][] P = {{.035, .04},
										{.04, .035}};
	
	public static final double[][] I = {{.001, .001},
										{.001, .001}};
	
	public static final double[][] D = {{.02, .02},
										{.02, .02}};
	
	private static final double LENGTH = 22.5;
	private static final double WIDTH = 20;

	private static final double TURN_ANGLE = Math.toDegrees(Math.atan2(WIDTH, LENGTH));

	private static final double[][] RELATIVE_ANGLES = {{TURN_ANGLE, 180-TURN_ANGLE},
													   {-TURN_ANGLE, TURN_ANGLE-180}};
	
	public static final double TURN_SPEED = .5;
	
	public static final int[][] drivePorts = {{0, 2},
											  {6, 4}};
	
	public static final double IN_DEAD = .1;

	public static final int[][] turnPorts = {{7, 1}, //Replaced 7 & 1
											 {5, 3}};
	
	private Spark[][] powerSparks = {{new Spark(drivePorts[0][0]), new Spark(drivePorts[0][1])},
							   		 {new Spark(drivePorts[1][0]), new Spark(drivePorts[1][1])}};

	private Spark[][] turnSparks = {{new Spark(turnPorts[0][0]),new Spark(turnPorts[0][1])},
								 	{new Spark(turnPorts[1][0]), new Spark(turnPorts[1][1])}};

	private int[][][] encPorts = {{{9, 8}, {7, 6}}, // invert ports?
								  {{0, 1}, {2, 3}}};

	private Encoder[][] encoders = {{new Encoder(encPorts[0][0][0], encPorts[0][0][1]), new Encoder(encPorts[0][1][0], encPorts[0][1][1])},
									{new Encoder(encPorts[1][0][0], encPorts[1][0][1]), new Encoder(encPorts[1][1][0], encPorts[1][1][1])}};

	private PIDController[][] pids = {{new PIDController(P[0][0], I[0][0], D[0][0], encoders[0][0], turnSparks[0][0]), new PIDController(P[0][1], I[0][1], D[0][1], encoders[0][1], turnSparks[0][1])},
									  {new PIDController(P[1][0], I[1][0], D[1][0], encoders[1][0], turnSparks[1][0]), new PIDController(P[1][1], I[1][1], D[1][1], encoders[1][1], turnSparks[1][1])}};

	private Joystick joy = new Joystick(0);
	
	private double[][] lastAngle = {{0, 0},
									{0, 0}};

	private AnalogGyro gyro;
	
	/* TODO:
	 * - Optionally get binary turning working on our joystick
	 * - Practice, Practice, Practice
	 */
	
	@Override
	public void robotInit() {
		
		gyro = new AnalogGyro(1);
		
		for (Encoder[] side: encoders) {
			for (Encoder encoder : side)
			{
				encoder.setReverseDirection(true);
				encoder.reset();
			}
		}
		
		for (PIDController[] side: pids)
		{
			for (PIDController pid: side)
			{
				pid.setOutputRange(-1, 1);
				pid.enable();
			}
		}
		
		for (Spark[] side: turnSparks)
		{
			for (Spark turnSpark: side)
			{
				turnSpark.setInverted(true);
			}
		}
	}

	private double round(double val)
	{
		return .01 * Math.round(val * 100);
	}
	
	private double angleToPulse(double angle) {
		return (206.0 / 180.0) * angle;
	}

	private double pulseToAngle(double pulse) {
		return (180.0 / 206.0) * pulse;
	}

	@Override
	public void teleopInit() {
		gyro.reset();
		
		System.out.println("This has started!");
		for (Encoder[] side : encoders) {
			for (Encoder encoder : side)
			{
				encoder.reset();
			}
		}
	}

	@Override
	public void teleopPeriodic() {
		if (joy.getTrigger())
		{
			pids[0][0].setSetpoint(gyro.getAngle());
			pids[0][1].setSetpoint(gyro.getAngle());
			pids[1][0].setSetpoint(gyro.getAngle());
			pids[1][1].setSetpoint(gyro.getAngle());
			lastAngle[0][0] = gyro.getAngle();
			lastAngle[0][1] = gyro.getAngle();
			lastAngle[1][0] = gyro.getAngle();
			lastAngle[1][1] = gyro.getAngle();
			gyro.reset();
		}
		
		double transMag = joy.getMagnitude();
		double rotMag = 0;
		
		if (joy.getAxisCount() > 2)
			rotMag = joy.getZ();
		else
			rotMag =  (joy.getRawButton(5) ? 1 : 0) - (joy.getRawButton(4) ? 1 : 0);
		
		if (Math.abs(rotMag) < IN_DEAD)
			rotMag = 0;
		
		rotMag *= TURN_SPEED;
		
		double transTheta = -gyro.getAngle();
		
		if (transMag > IN_DEAD)
			transTheta += joy.getDirectionDegrees();
		
		double transX = transMag * Math.sin(Math.toRadians(transTheta));
		double transY = transMag * Math.cos(Math.toRadians(transTheta));
		
		for (int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				double rotX = Math.sin(Math.toRadians(RELATIVE_ANGLES[i][j])) * rotMag;
				double rotY = Math.cos(Math.toRadians(RELATIVE_ANGLES[i][j])) * rotMag;
				double x = transX + rotX;
				double y = transY + rotY;
				double mag = Math.sqrt((x * x) + (y * y)) / Math.sqrt(2);
				double theta = Math.toDegrees(Math.atan2(x, y));
				theta = continufy(theta, lastAngle[i][j]);
				lastAngle[i][j] = theta;
				
				pids[i][j].setSetpoint(angleToPulse(theta));
				
				powerSparks[i][j].set(mag);
			}
		}
	}
	
	private double continufy(double theta, double prev)
	{
		double diff = theta - prev;
		
		while (diff > 180)
		{
			theta -= 360;
			diff = theta - prev;
		}
		
		while (diff < -180)
		{
			theta += 360;
			diff = theta - prev;
		}
		
		return theta;
	}
}
