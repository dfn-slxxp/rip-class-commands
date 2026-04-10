package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;

public class SwerveResetPoseKBShot extends SwerveResetPose {
    public SwerveResetPoseKBShot() {
        super(new Pose2d(0, Field.WIDTH / 2, CommandSwerveDrivetrain.getInstance().getPose().getRotation())); //TODO: FIELD CAL GET X
    }    
}
