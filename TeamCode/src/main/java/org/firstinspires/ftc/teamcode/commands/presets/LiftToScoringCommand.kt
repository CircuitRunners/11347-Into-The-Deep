package org.firstinspires.ftc.teamcode.commands.presets

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
import org.firstinspires.ftc.teamcode.subsystems.Slides
import org.firstinspires.ftc.teamcode.subsystems.Slides.SlidePositions

class LiftToScoringCommand(lift: Slides, preset: Presets) : ParallelCommandGroup() {

    enum class Presets {
        DOWN,
        AUTO,
        SHORT,
        MID,
        HIGH
    }

    init {
        addCommands(
            ParallelCommandGroup(
                SequentialCommandGroup(
                    // Change this ms to change when the arm comes up
                    WaitCommand(800),
                    InstantCommand({
                    }),
                ),
                InstantCommand({

                }),
                when (preset) {
                    Presets.DOWN ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_0.position, true)
                    Presets.AUTO ->
                        ProfiledLiftCommand(lift, SlidePositions.AUTO.position, true)
                    Presets.SHORT ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_1.position, true)
                    Presets.MID ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_2.position, true)
                    Presets.HIGH ->
                        ProfiledLiftCommand(lift, SlidePositions.STAGE_3.position, true)

                }
            )
        )
        addRequirements(lift)
    }
}
