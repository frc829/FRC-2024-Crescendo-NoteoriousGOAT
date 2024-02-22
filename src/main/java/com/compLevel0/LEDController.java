package com.compLevel0;

import java.util.function.Consumer;
import java.util.function.Function;

import com.ctre.phoenix.led.CANdle;
import com.types.animations.ctre.ColorFlow;
import com.types.animations.ctre.Fire;
import com.types.animations.ctre.Larson;
import com.types.animations.ctre.Rainbow;
import com.types.animations.ctre.RgbFade;
import com.types.animations.ctre.SingleFade;
import com.types.animations.ctre.Strobe;
import com.types.animations.ctre.Twinkle;
import com.types.animations.ctre.TwinkleOff;

public class LEDController {


    public final Function<Integer, Consumer<ColorFlow>> setColorFlowAnimation;
    public final Function<Integer, Consumer<Larson>> setLarsonAnimation;
    public final Function<Integer, Consumer<Strobe>> setStrobeAnimation;
    public final Function<Integer, Consumer<SingleFade>> setSingleFadeAnimation;
    public final Function<Integer, Consumer<Twinkle>> setTwinkleAnimation;
    public final Function<Integer, Consumer<TwinkleOff>> setTwinkleOffAnimation;
    public final Function<Integer, Consumer<Fire>> setFireAnimation;
    public final Function<Integer, Consumer<Rainbow>> setRainbowAnimation;
    public final Function<Integer, Consumer<RgbFade>> setRgbFadeAnimation;
    public final Consumer<Integer> clearAnimation;

    private LEDController(
            Function<Integer, Consumer<ColorFlow>> setColorFlowAnimation,
            Function<Integer, Consumer<Larson>> setLarsonAnimation,
            Function<Integer, Consumer<Strobe>> setStrobeAnimation,
            Function<Integer, Consumer<SingleFade>> setSingleFadeAnimation,
            Function<Integer, Consumer<Twinkle>> setTwinkleAnimation,
            Function<Integer, Consumer<TwinkleOff>> setTwinkleOffAnimation,
            Function<Integer, Consumer<Fire>> setFireAnimation,
            Function<Integer, Consumer<Rainbow>> setRainbowAnimation,
            Function<Integer, Consumer<RgbFade>> setRgbFadeAnimation,
            Consumer<Integer> clearAnimation) {
        this.setColorFlowAnimation = setColorFlowAnimation;
        this.setLarsonAnimation = setLarsonAnimation;
        this.setStrobeAnimation = setStrobeAnimation;
        this.setSingleFadeAnimation = setSingleFadeAnimation;
        this.setFireAnimation = setFireAnimation;
        this.setRainbowAnimation = setRainbowAnimation;
        this.setRgbFadeAnimation = setRgbFadeAnimation;
        this.setTwinkleAnimation = setTwinkleAnimation;
        this.setTwinkleOffAnimation = setTwinkleOffAnimation;
        this.clearAnimation = clearAnimation;
    }

    public static final Function<CANdle, LEDController> createFromCandle = (candle) -> {

        Function<Integer, Consumer<ColorFlow>> setColorFlowAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<Larson>> setLarsonAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<Strobe>> setStrobeAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<SingleFade>> setSingleFadeAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<Twinkle>> setTwinkleAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<TwinkleOff>> setTwinkleOffAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<Fire>> setFireAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<Rainbow>> setRainbowAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Function<Integer, Consumer<RgbFade>> setRgbFadeAnimation = (animationSlot) -> (animation) -> {
            candle.setLEDs(0, 0, 0, 0, animation.ledOffset, animation.numLed);
            candle.animate(animation.ctreVersion, animationSlot);
        };

        Consumer<Integer> clearAnimation = (animationSlot) -> {
            candle.clearAnimation(animationSlot);
        };

        return new LEDController(
                setColorFlowAnimation,
                setLarsonAnimation,
                setStrobeAnimation,
                setSingleFadeAnimation,
                setTwinkleAnimation,
                setTwinkleOffAnimation,
                setFireAnimation,
                setRainbowAnimation,
                setRgbFadeAnimation,
                clearAnimation);
    };

}
