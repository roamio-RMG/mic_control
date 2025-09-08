#include <stdio.h>
#include <stdlib.h>
int main(int ac, char *av[]) {
    int angle = atoi(av[1]);
    int angle_to_turn = 0;
    if (angle > 295) {
        angle_to_turn = 270 - angle;
        printf("Voice detected on left - rotating LEFT to face speaker: %d\n", angle_to_turn);
    }
    else if (angle > 0 && angle <= 90) {
        angle_to_turn = angle + 90;
        angle_to_turn *= -1;
        printf("Voice detected on left - rotating LEFT to face speaker: %d\n", angle_to_turn);
    }
    else if (angle > 90 && angle <= 247) {
        angle_to_turn = 270 - angle;
        printf("Voice detected on right - rotating RIGHT to face speaker: %d\n", angle_to_turn);
    }
    else {
        printf("Voice detected in front - staying still: %d\n", angle_to_turn);
    }
}

