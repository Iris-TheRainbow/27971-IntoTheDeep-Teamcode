package org.firstinspires.ftc.teamcode.commandbase.subsystems;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commandbase.Vision;
import org.firstinspires.ftc.vision.VisionPortal;

import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import dev.frozenmilk.dairy.core.dependency.Dependency;
import dev.frozenmilk.dairy.core.dependency.annotation.SingleAnnotation;
import dev.frozenmilk.dairy.core.wrapper.Wrapper;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.mercurial.subsystems.Subsystem;
import kotlin.annotation.MustBeDocumented;

public class SubAuto implements Subsystem {
    public static final SubAuto INSTANCE = new SubAuto();

    private SubAuto() { }

    private static Vision vision;
    private static VisionPortal visionPortal;
    private static Vision.TeamColor teamColor = new Vision.NullAlliance();
    private static Telemetry telem;
    private static HardwareMap hwmap;
    private static double xOffset = 0;
    private static double yOffset = 0;
    private static double clawAngle;
    private static void buildVision(Vision.TeamColor teamColor){
        SubAuto.teamColor = teamColor;
        vision = new Vision(teamColor);
        if (visionPortal != null){
            visionPortal.close();
            visionPortal = null;
        }
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "cam"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(vision)
                .build();
    }
    @Override
    public void postUserInitHook(@NonNull Wrapper opMode) {
        hwmap = opMode.getOpMode().hardwareMap;
        telem = opMode.getOpMode().telemetry;
        buildVision(teamColor());
    }

    @Override
    public void postUserInitLoopHook(@NonNull Wrapper OpMode){
        telem.addLine(teamColor.toString());
    }
    @Override
    public void postUserStopHook(@NonNull Wrapper opMode){
        visionPortal.close();
    }
    public static DoubleSupplier xOffset(){
        return () -> vision.getClosestXInches();
    }
    public static DoubleSupplier yOffset(){
        return () -> vision.getClosestYInches();
    }
    public static double clawAngle(){
        return clawAngle;
    }
    public static Vision.TeamColor teamColor(){
        return teamColor;
    }
    @NonNull
    public static Lambda setRed() {
        return setColor(new Vision.RedAlliance());
    }
    @NonNull
    public static Lambda setBlue() {
        return setColor(new Vision.BlueAlliance());
    }

    @NonNull
    public static Command doVision() {
        return new Sequential(new Wait(.1),
                new Lambda("DoVision")
                        .setInit(() -> {
                            xOffset = vision.getClosestXInches() + 1;
                            yOffset = vision.getClosestYInches();
                            clawAngle = vision.getClosestRotate();
                        }));
    }

    @NonNull
    public static Lambda setColor(Vision.TeamColor color) {
        return new Lambda("setRed")
                .setRunStates(Wrapper.OpModeState.INIT)
                .addRequirements(INSTANCE)
                .setInit(() -> buildVision(color));
    }

    @Retention(RetentionPolicy.RUNTIME) @Target(ElementType.TYPE) @MustBeDocumented
    @Inherited
    public @interface Attach { }

    private Dependency<?> dependency = Subsystem.DEFAULT_DEPENDENCY.and(new SingleAnnotation<>(Attach.class));

    @NonNull
    @Override
    public Dependency<?> getDependency() { return dependency; }

    @Override
    public void setDependency(@NonNull Dependency<?> dependency) { this.dependency = dependency; }
}