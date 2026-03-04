package com.stuypulse.robot.commands.vision;

import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetIMUAssistValue extends InstantCommand {
    private LimelightVision vision;
    private double assistValue;

    public SetIMUAssistValue(double assistValue) {
        vision = LimelightVision.getInstance();
    }

    @Override
    public void initialize() {
        vision.setIMUAssistValue(assistValue);
    }
}
