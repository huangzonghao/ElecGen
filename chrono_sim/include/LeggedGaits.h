#ifndef LEGGEDGAITS_H_
#define LEGGEDGAITS_H_

void WheelController::exe_gait(){
    // motor 0 1 left, 2 3 right
    // negative vel moves the robot forward
    switch(gait){
        case FORWARD:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
        case BACKWARD:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
        case RIGHT1:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
        case LEFT1:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
        case RIGHT2:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
        case LEFT2:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
    }
}

void LeggedController::exe_gait(){
    // motor 0 1 left, 2 3 right
    // positive vel moves the robot forward
    // // after turning, sync legs to the same phase
    // if (gait != FORWARD && gait != BACKWARD){
        // for (auto motor : *motors_){
            // motor->SetPhase(1.5);
        // }
        // // wait until all the legs are in the right phase
        // gait_lock = true;
        // gait = FORWARD;
        // return false;
    // }
    switch(gait){
        case STAND:
            motors_->at(0)->SetVel(1.5);
            motors_->at(1)->SetVel(1.5);
            motors_->at(2)->SetVel(1.5);
            motors_->at(3)->SetVel(1.5);
            break;
        case FORWARD:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
        case BACKWARD:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
        case LEFT1:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
        case RIGHT1:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
        case LEFT2:
            motors_->at(0)->SetVel(6);
            motors_->at(1)->SetVel(6);
            motors_->at(2)->SetVel(-6);
            motors_->at(3)->SetVel(-6);
            break;
        case RIGHT2:
            motors_->at(0)->SetVel(-6);
            motors_->at(1)->SetVel(-6);
            motors_->at(2)->SetVel(6);
            motors_->at(3)->SetVel(6);
            break;
    }
}

void LeggedController::exe_gait2(){
    // motor pair sequence: fl fr bl br
    // positive vel moves the robot forward
    switch(gait){
        case STAND:
            motors_->at(0)->SetPhase(-0.3);
            motors_->at(1)->SetPhase(0.7);
            motors_->at(2)->SetPhase(-0.3);
            motors_->at(3)->SetPhase(0.7);
            motors_->at(4)->SetPhase(-0.3);
            motors_->at(5)->SetPhase(0.7);
            motors_->at(6)->SetPhase(-0.3);
            motors_->at(7)->SetPhase(0.7);
            break;
        case FORWARD:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(-0.3);
                    motors_->at(1)->SetPhase(0.7);
                    motors_->at(2)->SetPhase(-0.3);
                    motors_->at(3)->SetPhase(0.7);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(-0.3);
                    motors_->at(7)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(-0.3);
                    motors_->at(1)->SetPhase(0.7);
                    motors_->at(2)->SetPhase(-0.3);
                    motors_->at(3)->SetPhase(0.7);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(-0.3);
                    motors_->at(7)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(-0.3);
                    motors_->at(1)->SetPhase(0.7);
                    motors_->at(2)->SetPhase(-0.7);
                    motors_->at(3)->SetPhase(1);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(1);
                    motors_->at(6)->SetPhase(-0.3);
                    motors_->at(7)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(-0.7);
                    motors_->at(1)->SetPhase(1);
                    motors_->at(2)->SetPhase(-0.3);
                    motors_->at(3)->SetPhase(0.7);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(-0.7);
                    motors_->at(7)->SetPhase(1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case BACKWARD:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0.3);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.3);
                    motors_->at(3)->SetPhase(-0.7);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0.3);
                    motors_->at(7)->SetPhase(-0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(0.3);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.3);
                    motors_->at(3)->SetPhase(-0.7);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0.3);
                    motors_->at(7)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(0.3);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(-1);
                    motors_->at(4)->SetPhase(0.7);
                    motors_->at(5)->SetPhase(-1);
                    motors_->at(6)->SetPhase(0.3);
                    motors_->at(7)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(0.7);
                    motors_->at(1)->SetPhase(-1);
                    motors_->at(2)->SetPhase(0.3);
                    motors_->at(3)->SetPhase(-0.7);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0.7);
                    motors_->at(7)->SetPhase(-1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
    }
}

void LeggedController::exe_gait3(){
    // fl bl fr br
    switch(gait){
        case STAND:
            motors_->at(0)->SetPhase(0);
            motors_->at(1)->SetPhase(-0.3);
            motors_->at(2)->SetPhase(0.7);
            motors_->at(3)->SetPhase(0);
            motors_->at(4)->SetPhase(-0.3);
            motors_->at(5)->SetPhase(0.7);
            motors_->at(6)->SetPhase(0);
            motors_->at(7)->SetPhase(-0.3);
            motors_->at(8)->SetPhase(0.7);
            motors_->at(9)->SetPhase(0);
            motors_->at(10)->SetPhase(-0.3);
            motors_->at(11)->SetPhase(0.7);
            break;
        case FORWARD:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(1);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.7);
                    motors_->at(8)->SetPhase(1);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(1);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.7);
                    motors_->at(11)->SetPhase(1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case BACKWARD:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(0.3);
                    motors_->at(2)->SetPhase(-0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(0.3);
                    motors_->at(8)->SetPhase(-0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(0.3);
                    motors_->at(11)->SetPhase(-0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(0.3);
                    motors_->at(2)->SetPhase(-0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(0.3);
                    motors_->at(8)->SetPhase(-0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(0.3);
                    motors_->at(11)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(0.3);
                    motors_->at(2)->SetPhase(-0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(0.7);
                    motors_->at(5)->SetPhase(-1);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(0.7);
                    motors_->at(8)->SetPhase(-1);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(0.3);
                    motors_->at(11)->SetPhase(-0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(0.7);
                    motors_->at(2)->SetPhase(-1);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(0.3);
                    motors_->at(5)->SetPhase(-0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(0.3);
                    motors_->at(8)->SetPhase(-0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(0.7);
                    motors_->at(11)->SetPhase(-1);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
            }
            break;
        case LEFT1:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0.8);
                    motors_->at(10)->SetPhase(-0.7);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0.8);
                    motors_->at(7)->SetPhase(-0.7);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case RIGHT1:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(-0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(-0.8);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(-0.8);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.7);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.7);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case LEFT2:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0.8);
                    motors_->at(10)->SetPhase(-0.7);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0.8);
                    motors_->at(7)->SetPhase(-0.7);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0.8);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors_->at(0)->SetPhase(0.8);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
        case RIGHT2:
            switch(gait_steps){
                case 0:
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    motors_->at(6)->SetPhase(0);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    gait_steps = 4;
                    gait_lock = true;
                    break;
                case 1:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(-0.8);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(-0.8);
                    motors_->at(4)->SetPhase(-0.7);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 0;
                    gait_lock = true;
                    break;
                case 2:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(-0.8);
                    motors_->at(1)->SetPhase(-0.7);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 1;
                    gait_lock = true;
                    break;
                case 3:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.3);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(-0.8);
                    motors_->at(10)->SetPhase(-0.7);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 2;
                    gait_lock = true;
                    break;
                case 4:
                    motors_->at(6)->SetPhase(-0.8);
                    motors_->at(7)->SetPhase(-0.7);
                    motors_->at(8)->SetPhase(0.7);
                    motors_->at(9)->SetPhase(0);
                    motors_->at(10)->SetPhase(-0.3);
                    motors_->at(11)->SetPhase(0.7);
                    motors_->at(0)->SetPhase(0);
                    motors_->at(1)->SetPhase(-0.3);
                    motors_->at(2)->SetPhase(0.7);
                    motors_->at(3)->SetPhase(0);
                    motors_->at(4)->SetPhase(-0.3);
                    motors_->at(5)->SetPhase(0.7);
                    gait_steps = 3;
                    gait_lock = true;
                    break;
            }
            break;
    }

}


#endif /* end of LEGGEDGAITS_H_ */
