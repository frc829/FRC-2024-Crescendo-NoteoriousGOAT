package com.types.animations.ctre;

import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

public class Twinkle {

    public final int r;
    public final int g;
    public final int b;
    public final int w;
    public final double speed;
    public final int numLed;
    public final double divider;
    public final int ledOffset;
    public final TwinkleAnimation ctreVersion;

    public Twinkle(
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
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent6, ledOffset);
        } else if (divider <= 18) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent18, ledOffset);
        } else if (divider <= 30) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent30, ledOffset);
        } else if (divider <= 42) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent42, ledOffset);
        } else if (divider <= 64) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent64, ledOffset);
        } else if (divider <= 76) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent76, ledOffset);
        } else if (divider <= 76) {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent88, ledOffset);
        } else {
            ctreVersion = new TwinkleAnimation(r, g, b, w, speed, numLed, TwinklePercent.Percent100, ledOffset);
        }
    }

}
