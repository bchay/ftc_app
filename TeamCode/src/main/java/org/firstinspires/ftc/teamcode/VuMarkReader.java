package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuMarkReader {
    private VuforiaTrackable relicTemplate;
    private VuforiaLocalizer vuforia;

    public VuMarkReader(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AcSmd8X/////AAAAGY0Yqr1v4EYCrKeT+OkWoBQ5q0bkTU0tTzZ+CXBHFr9bALTHzyDlh8FtKKyPmJbF8mtacz2bLRrwvgDcGWGzlqWGdET+qipWSFq1aRJy02jB7rEtnOdJV6mCyzUErz55VFh8DTFW3A373oHemYSnOQL63h88FknvhskKiGckmYIX3vhVpj3QvEpEL4oChVlnmr4AGLfhTyFLQ9C1R7iNxOSP8FAOzDslaDWJ6jT4iaD4E96jNgxhjXaWxmOqRW4MeXXmBESdV45cvxbGAlfwv6cp/zB8b+w8AI4YpwwbGDqPT4nimANFBQ3zAJTlZTDDFeyVbA0bQIWQuIi+ZACA/qloSYaNOYOkelXQ+JTj0dFv";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTrackables.activate();
    }

    public RelicRecoveryVuMark getVuMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }
}