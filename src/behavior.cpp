#include "behavior.h"
#include "state.h"
#include <iostream>

using namespace std;

Behavior::Behavior() : 
    ready_(State::READY),
    keep_lane_(State::LANE_KEEP),
    lane_change_left_(State::LANE_CHANGE_LEFT),
    lane_change_right_(State::LANE_CHANGE_RIGHT),
    prepare_lane_change_left_(State::PREPARE_LANE_CHANGE_LEFT),
    prepare_lane_change_right_(State::PREPARE_LANE_CHANGE_RIGHT)
{
    // ready transitions
    ready_.addTransition(&ready_);
    ready_.addTransition(&keep_lane_);

    // keep lane transitions
    keep_lane_.addTransition(&keep_lane_);
    keep_lane_.addTransition(&prepare_lane_change_left_);
    keep_lane_.addTransition(&prepare_lane_change_right_);

    // prepare lane change left transitions
    prepare_lane_change_left_.addTransition(&keep_lane_);
    prepare_lane_change_left_.addTransition(&lane_change_left_);
    prepare_lane_change_left_.addTransition(&prepare_lane_change_right_);

    // prepare lane change right transitions
    prepare_lane_change_right_.addTransition(&keep_lane_);
    prepare_lane_change_right_.addTransition(&lane_change_right_);
    prepare_lane_change_right_.addTransition(&prepare_lane_change_right_);

    // change lane left transitions
    lane_change_left_.addTransition(&lane_change_left_);
    lane_change_left_.addTransition(&keep_lane_);

    // change lane right transitions
    lane_change_right_.addTransition(&lane_change_right_);
    lane_change_right_.addTransition(&keep_lane_);

    // // Set initial state
    this->state_ = &ready_;
}

Behavior::~Behavior() {}

void Behavior::transition() {
    vector<State*> states = state_->getTransitions();
    cout << "Current ID: " << state_->getId() << endl;
    for (int i = 0; i < states.size(); i++) {
        cout << "ID: " << states[i]->getId() << endl;
    }
}


