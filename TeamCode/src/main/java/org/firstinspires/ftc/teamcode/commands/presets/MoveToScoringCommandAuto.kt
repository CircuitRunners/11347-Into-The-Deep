//package org.firstinspires.ftc.teamcode.commands.presets
//
//import com.arcrobotics.ftclib.command.InstantCommand
//import com.arcrobotics.ftclib.command.ParallelCommandGroup
//import com.arcrobotics.ftclib.command.SequentialCommandGroup
//import com.arcrobotics.ftclib.command.WaitCommand
//import org.firstinspires.ftc.teamcode.commands.liftcommands.ProfiledLiftCommand
//import org.firstinspires.ftc.teamcode.subsystems.Outtake
//import org.firstinspires.ftc.teamcode.subsystems.Slides
//import org.firstinspires.ftc.teamcode.subsystems.Slides.SlidePositions
//import org.firstinspires.ftc.teamcode.subsystems.V4B
//
//class MoveToScoringCommandAuto(lift: Slides, v4b: V4B, outtake: Outtake, preset: Presets) : ParallelCommandGroup() {
//
//    enum class Presets {
//        AUTO
//    }
//
//    init {
//        addCommands(
//                ParallelCommandGroup(
//                        SequentialCommandGroup(
//                                // Change this ms to change when the arm comes up
//                                WaitCommand(800),
//                                InstantCommand({
////                                    v4b.extend()
//                                }),
//                        ),
//                        InstantCommand({
//                            outtake.transport()
//                        }),
//                        when (preset) {
//                            Presets.AUTO ->
//                                ProfiledLiftCommand(lift, SlidePositions.AUTO.position, true)
//
//                        }
//                )
//        )
//        addRequirements(lift)
//    }
//}