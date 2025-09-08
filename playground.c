#include <stdio.h>
#include <stdlib.h>
int main(int ac, char *av[]) {
    int angle = atoi(av[1]);
    int angle_to_move = 0;
    if (angle > 295) {
        angle_to_move = 270 - angle;
        printf("Voice detected on left - rotating LEFT to face speaker: %d\n", angle_to_move);
    }
    else if (angle > 0 && angle <= 90) {
        angle_to_move = angle + 90;
        angle_to_move *= -1;
        printf("Voice detected on left - rotating LEFT to face speaker: %d\n", angle_to_move);
    }
    else if (angle > 90 && angle <= 247) {
        angle_to_move = 270 - angle;
        printf("Voice detected on right - rotating RIGHT to face speaker: %d\n", angle_to_move);
    }
    else {
        printf("Voice detected in front - staying still: %d\n", angle_to_move);
    }
    
    int angle_to_turn = 0;

    if  (angle < 247 || angle > 295)
    {
        if (angle_to_move < 0) {
            angle_to_turn = angle_to_move + 360;
        }
        else {
            angle_to_turn = angle_to_move;
        }
        printf("--------------------------------\n");
        printf("Angle to turn: %d\n", angle_to_turn);
    }

    // angle_to_turn = 0
    // if  angle < 247 and angle > 295:
    //     if angle_to_move < 0:
    //         angle_to_turn = angle_to_move + 360
    //     else:
    //         angle_to_turn = angle_to_move
            
    
}

