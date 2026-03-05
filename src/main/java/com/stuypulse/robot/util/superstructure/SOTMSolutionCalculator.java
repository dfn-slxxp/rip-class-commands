/************************ PROJECT ALPHA *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package com.stuypulse.robot.util.superstructure;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.util.superstructure.ShotCalculator.SOTMSolution;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SOTMSolutionCalculator {
    
    public static SOTMSolution sol;

    private static FieldObject2d hubPose2d;
    private static FieldObject2d virtualHubPose2d;
    private static FieldObject2d futureTurretPose2d;


    static {
        sol = new SOTMSolution(
            Hood.getInstance().getAngle(),
            Turret.getInstance().getAngle(),
            Shooter.getInstance().getRPM(), 
            Field.getHubPose(), 
            0.0
        );

        hubPose2d = Field.FIELD2D.getObject("hubPose");
        virtualHubPose2d = Field.FIELD2D.getObject("virtualHubPose");
        futureTurretPose2d = Field.FIELD2D.getObject("futureTurretPose");
    }


    public static void updateSOTMSolution() {

        CommandSwerveDrivetrain swerve = CommandSwerveDrivetrain.getInstance();
        
        Pose2d robotPose = swerve.getPose();
        Pose2d hubPose = Field.getHubPose();
        
        ChassisSpeeds robotRelativeSpeeds = swerve.getChassisSpeeds();
        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds, 
            robotPose.getRotation()
        );

        Pose2d futureTurretPose = swerve.getTurretPose().exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue(),
                robotRelativeSpeeds.vyMetersPerSecond * Settings.Superstructure.SOTM.UPDATE_DELAY.doubleValue(),
                0
            )
        );

        Vector2D oppositeDirection = new Vector2D(new Translation2d(
            -robotRelativeSpeeds.vxMetersPerSecond,
            -robotRelativeSpeeds.vyMetersPerSecond
        ));

        if (!oppositeDirection.equals(Vector2D.kOrigin)) {
            oppositeDirection = oppositeDirection.normalize();
        }

        hubPose = hubPose.exp(
            new Twist2d(
                oppositeDirection.x * Field.HUB_RADIUS,
                oppositeDirection.y * Field.HUB_RADIUS,
                0
            )
        );
        

        SOTMSolution solution = ShotCalculator.solveShootOnTheMove(
            futureTurretPose,
            robotPose,
            hubPose,
            fieldRelativeSpeeds,
            Settings.Superstructure.SOTM.MAX_ITERATIONS,
            Settings.Superstructure.SOTM.TIME_TOLERANCE
        );

        sol = solution;

        hubPose2d.setPose(Robot.isBlue() ? hubPose : Field.transformToOppositeAlliance(hubPose));
        virtualHubPose2d.setPose((Robot.isBlue() ? sol.virtualPose() : Field.transformToOppositeAlliance(sol.virtualPose())));
        futureTurretPose2d.setPose((Robot.isBlue() ? futureTurretPose : Field.transformToOppositeAlliance(futureTurretPose)));
  
  
        SmartDashboard.putNumber("Superstructure/SOTM/calculated turret angle", sol.targetTurretAngle().getDegrees());
        SmartDashboard.putNumber("Superstructure/SOTM/calculated hood angle", sol.targetHoodAngle().getDegrees());
        SmartDashboard.putNumber("Superstructure/SOTM/calculated flight time", sol.flightTime());
        SmartDashboard.putNumber("Superstructure/SOTM/turret dist to virtual pose", futureTurretPose.getTranslation().getDistance(sol.virtualPose().getTranslation()));
    }

    public static Supplier<Rotation2d> calculateHoodAngleSOTM() {
        return () -> sol.targetHoodAngle();
    }
    
    public static Supplier<Rotation2d> calculateTurretAngleSOTM() {
        return () -> sol.targetTurretAngle();
    }
    
    public static Supplier<Double> calculateShooterRPMSOTM() {
        return () -> sol.targetShooterRPM();
    }
}