package org.firstinspires.ftc.teamcode.util;

import java.util.Arrays;

public class telemUtil {
    public static String slider(int length, int marker){
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < length; i++) {
            if (i == marker){
                out.append("X");
            } else{
                out.append("-");
            }
        }

        return out.toString();
    }
}
