#ifndef CONSTANT_H
#define CONSTANT_H

enum mode{
    FOLLOW_NEAREST_PEOPLE,
    FOLLOW_TALKING_PEOPLE
};

enum actionState{
    IDLE_STATE,
    STATE_1,
    STATE_2,
    STATE_3,
    STATE_4,
    END_STATE,
    ACTION_SIZE = END_STATE +1
};

#endif