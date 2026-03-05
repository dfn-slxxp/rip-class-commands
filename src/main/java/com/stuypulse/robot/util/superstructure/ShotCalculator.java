/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator.InterpolatedShotInfo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class ShotCalculator {
    public static final double g = 9.81;


    public record SOTMSolution(
        Rotation2d targetHoodAngle,
        Rotation2d targetTurretAngle,
        double targetShooterRPM,
        Pose2d virtualPose,
        double flightTime) {
    }

    public static SOTMSolution solveShootOnTheMove(
        Pose2d turretPose,
        Pose2d robotPose,
        Pose2d targetPose,
        ChassisSpeeds fieldRelativeSpeeds,
        int maxIterations,
        double timeTolerance) {

        /*
        Start with v_ball * flightTime = distanceToTargetPose.
        
        We know that v_ball = v_robot + v_shooter, so 
        (v_robot + v_shooter) * flightTime = distanceToTargetPose

        Rearranging, we can get
        (v_shooter) * flight_time = distanceToTargetPose - v_robot * flightTime

        So we can instead shoot at a virtual pose and treat the robot as stationary:
        distanceToVirtualPose = distanceToTargetPose - v_robot * flightTime
        (v_shooter) * flight_time = distanceToVirtualPose

        Looking at the first equation, we can find the virtual pose with the flight time, 
        but looking at the second equation, to get the flight time we need to solveBallisticWithSpeed()
        using the virtual pose, so we have a circular dependence.

        Thus, we can make an initial guess for the flight time: the flight time if the robot were stationary
        We want our guess to converge such that the left side equals the right side:
        (v_shooter) * t_guess = distanceToVirtualPose = distance - v_robot * t_guess, which would make t_guess = flightTime

        We do the right side first using our inital guess, and then update t_guess with a new guess by 
        calculating the flightTime to that virtualPose.

        The pose is that the flightTime converges within maxIterations.
        */
        

        InterpolatedShotInfo sol = InterpolationCalculator.interpolateShotInfo();

        
        double t_guess = sol.flightTimeSeconds();
        
        Pose2d virtualPose = targetPose;

             
        for (int i = 0; i < maxIterations; i++) {

            SmartDashboard.putNumber("Superstructure/SOTM/iteration #", i);

            double dx = fieldRelativeSpeeds.vxMetersPerSecond * t_guess;
            double dy = fieldRelativeSpeeds.vyMetersPerSecond * t_guess;

            virtualPose = new Pose2d(
                targetPose.getX() - dx,
                targetPose.getY() - dy,
                targetPose.getRotation());

  
            InterpolatedShotInfo newSol = InterpolationCalculator.interpolateShotInfo(virtualPose);

            if (Math.abs(newSol.flightTimeSeconds() - t_guess) < timeTolerance) {
                break;
            }

            t_guess = newSol.flightTimeSeconds();

            sol = newSol;

        }
        
        Translation2d virtualTranslation = virtualPose.getTranslation();
        Translation2d turretTranslation = turretPose.getTranslation();

        double yaw = Math.atan2(
            virtualTranslation.getY() - turretTranslation.getY(),
            virtualTranslation.getX() - turretTranslation.getX() 
        );
        
        Rotation2d targetTurretAngle = Robot.isReal() ? 
            Rotation2d.fromRadians(-yaw).plus(robotPose.getRotation()) :
            Rotation2d.fromRadians(yaw).minus(robotPose.getRotation());

        return new SOTMSolution(
            sol.targetHoodAngle(),
            targetTurretAngle,
            sol.targetRPM(),
            virtualPose,
            sol.flightTimeSeconds()
        );
    }
}