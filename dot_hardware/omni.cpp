#include "motor.h"

int main(){
    Motor a(pi_22, pi_23, pi_27, pi_17, pi_18, 10f, 100f, 0f);

    // while(true){
    //     softPwmWrite(pi_27, 100);
    //     if((count/10000)%2){
    //         digitalWrite(pi_22, HIGH);
    //         digitalWrite(pi_23, LOW);
    //     } else {
    //         digitalWrite(pi_22, LOW);
    //         digitalWrite(pi_23, HIGH);
    //     }
    //     count++;
    // }
    return 0;
}