#include "Timer.h"

Timer::Timer(unsigned long duration) {
    timerDuration = duration;
}

void Timer::update() {
    lastUpdate = micros();
}

bool Timer::timeHasPassed() {
    if (micros() - lastUpdate > timerDuration) {
        update();
        return true;
    }

    return false;
}

bool Timer::timeHasPassedNoUpdate() {
    return micros() - lastUpdate > timerDuration;
}

void Timer::resetTime() {
    lastUpdate = micros();
}
