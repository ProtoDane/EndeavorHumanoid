#ifndef ACTIONS_H
#define ACTIONS_H

#include <Bluepad32.h>

#include "presets.h"
#include "systemParams.h"
#include "servo2040.h"
#include "kinematics_solver.h"




class actionHandler {
    public:
        actionHandler();

        void begin(servoHandler servo, QueueHandle_t imuQueue);

        void moveWalkFwd(ControllerPtr gamepad);
        void moveWalkBwd(ControllerPtr gamepad);
        void moveStrafe(ControllerPtr gamepad, bool dir);
        void moveSpin(ControllerPtr, bool dir);

        void actionIdle();
        void actionCrouch(ControllerPtr gamepad);
        void actionGetup(ControllerPtr gamepad);
        void actionPunch(ControllerPtr gamepad, bool dir);
        void actionUpperCut(ControllerPtr gamepad, bool dir);
        void actionSwipe(ControllerPtr gamepad, bool  dir);
        void actionTumble(ControllerPtr gamepad);

        void emoteShikanoko(ControllerPtr gamepad);
        void emoteHero(ControllerPtr gamepad);
        void emoteTaunt(ControllerPtr gamepad);
        void emoteGangnam(ControllerPtr gamepad);

    private:
        QueueHandle_t _imuQueue;
        servoHandler _servo;
    
        void fetchQueue(queueBin *q);
};

#endif