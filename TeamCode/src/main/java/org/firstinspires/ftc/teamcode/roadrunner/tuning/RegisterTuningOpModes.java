package org.firstinspires.ftc.teamcode.roadrunner.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.roadrunner.SparkFunOTOSDrive;

import java.util.Arrays;

import dev.frozenmilk.sinister.opmode.SinisterOpModes;


@TeleOp
public class RegisterTuningOpModes extends OpMode {
    public final Class<?> DRIVE_CLASS = SparkFunOTOSDrive.class;

    public final String GROUP = "quickstart";
    public final boolean DISABLED = false;


    private OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    public void register() {

        SinisterOpModes.INSTANCE.register(metaForClass(AngularRampLogger.class), AngularRampLogger.class);
        SinisterOpModes.INSTANCE.register(metaForClass(ForwardRampLogger.class), ForwardRampLogger.class);
        SinisterOpModes.INSTANCE.register(metaForClass(LateralRampLogger.class), LateralRampLogger.class);
        SinisterOpModes.INSTANCE.register(metaForClass(ManualFeedforwardTuner.class), ManualFeedforwardTuner.class);
        SinisterOpModes.INSTANCE.register(metaForClass(MecanumMotorDirectionDebugger.class), MecanumMotorDirectionDebugger.class);
        SinisterOpModes.INSTANCE.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        SinisterOpModes.INSTANCE.register(metaForClass(SplineTest.class), SplineTest.class);
        SinisterOpModes.INSTANCE.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
    @Override
    public void init() {

    }
    @Override
    public void start(){
        register();
        requestOpModeStop();
    }
    @Override
    public void loop() {
        requestOpModeStop();
    }
}
