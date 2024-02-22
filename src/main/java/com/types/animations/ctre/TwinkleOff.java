package com.types.animations.ctre;

import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class TwinkleOff {

    public final int r;
    public final int g;
    public final int b;
    public final int w;
    public final double speed;
    public final int numLed;
    public final double divider;
    public final int ledOffset;
    public final TwinkleOffAnimation ctreVersion;

    public TwinkleOff(
            int r,
            int g,
            int b,
            int w,
            double speed,
            int numLed,
            double divider,
            int ledOffset) {
        this.r = r;
        this.g = g;
        this.b = b;
        this.w = w;
        this.speed = speed;
        this.numLed = numLed;
        this.divider = divider;
        this.ledOffset = ledOffset;

        if (divider <= 6) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent6, ledOffset);
        } else if (divider <= 18) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent18, ledOffset);
        } else if (divider <= 30) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent30, ledOffset);
        } else if (divider <= 42) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent42, ledOffset);
        } else if (divider <= 64) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent64, ledOffset);
        } else if (divider <= 76) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent76, ledOffset);
        } else if (divider <= 76) {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent88, ledOffset);
        } else {
            ctreVersion = new TwinkleOffAnimation(r, g, b, w, speed, numLed, TwinkleOffPercent.Percent100, ledOffset);
        }
    }

}
