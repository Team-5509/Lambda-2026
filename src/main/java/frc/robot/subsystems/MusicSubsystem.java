// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.util.List;

/**
 * Subsystem for playing CHRP music files through TalonFX motors using CTRE's Orchestra API.
 * 
 * <p>Important notes:
 * <ul>
 *   <li>Robot must be ENABLED to play music (by default)</li>
 *   <li>Use a free-spinning motor or put robot on blocks to prevent movement</li>
 *   <li>Each TalonFX plays one track - add multiple instruments for multi-track songs</li>
 *   <li>More motors = louder and fuller sound</li>
 * </ul>
 */
public class MusicSubsystem extends SubsystemBase {
    private final Orchestra orchestra = new Orchestra();

    /**
     * Creates a new MusicSubsystem with a single motor.
     * 
     * @param talonId CAN ID of the Kraken/TalonFX motor to use for music
     * @param canbus CAN bus name ("rio" for roboRIO bus, or CANivore name)
     * @param chrpFileName Path to CHRP file relative to deploy directory (e.g., "songs/yourSong.chrp")
     */
    public MusicSubsystem(int talonId, String canbus, String chrpFileName) {
        TalonFX musicMotor = new TalonFX(talonId, canbus);

        // Add the Kraken motor as an instrument
        orchestra.addInstrument(musicMotor);

        // Load CHRP file from deploy directory
        File song = new File(Filesystem.getDeployDirectory(), chrpFileName);
        orchestra.loadMusic(song.getAbsolutePath());
    }

    /**
     * Creates a new MusicSubsystem with multiple motors for louder/fuller sound.
     * 
     * @param motors List of TalonFX motors to use for music (e.g., all drivetrain motors)
     * @param chrpFileName Path to CHRP file relative to deploy directory (e.g., "songs/yourSong.chrp")
     */
    public MusicSubsystem(List<TalonFX> motors, String chrpFileName) {
        // Add all motors as instruments
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }

        // Load CHRP file from deploy directory
        File song = new File(Filesystem.getDeployDirectory(), chrpFileName);
        orchestra.loadMusic(song.getAbsolutePath());
    }

    /**
     * Starts playing the loaded CHRP file.
     */
    public void play() {
        orchestra.play();
    }

    /**
     * Pauses the currently playing song.
     */
    public void pause() {
        orchestra.pause();
    }

    /**
     * Stops the currently playing song.
     */
    public void stop() {
        orchestra.stop();
    }

    /**
     * Checks if music is currently playing.
     * 
     * @return true if music is playing, false otherwise
     */
    public boolean isPlaying() {
        return orchestra.isPlaying();
    }
}
