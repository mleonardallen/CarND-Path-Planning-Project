#include "behavior.h"
#include "state.h"
#include <iostream>

using namespace std;

Behavior::Behavior() {

    // initialize states
    State ready;
    ready.setId(State::READY);

    State keep_lane;
    keep_lane.setId(State::LANE_KEEP);

    State lane_change_left;
    lane_change_left.setId(State::LANE_CHANGE_LEFT);

    State lane_change_right;
    lane_change_right.setId(State::LANE_CHANGE_RIGHT);

    State prepare_lane_change_left;
    prepare_lane_change_left.setId(State::PREPARE_LANE_CHANGE_LEFT);

    State prepare_lane_change_right;
    prepare_lane_change_right.setId(State::PREPARE_LANE_CHANGE_RIGHT);

    // ready transitions
    ready.addTransition(&ready);
    ready.addTransition(&keep_lane);

    // keep lane transitions
    keep_lane.addTransition(&keep_lane);
    keep_lane.addTransition(&prepare_lane_change_left);
    keep_lane.addTransition(&prepare_lane_change_right);

    // prepare lane change left transitions
    prepare_lane_change_left.addTransition(&keep_lane);
    prepare_lane_change_left.addTransition(&lane_change_left);
    prepare_lane_change_left.addTransition(&prepare_lane_change_right);

    // prepare lane change right transitions
    prepare_lane_change_right.addTransition(&keep_lane);
    prepare_lane_change_right.addTransition(&lane_change_right);
    prepare_lane_change_right.addTransition(&prepare_lane_change_right);

    // change lane left transitions
    lane_change_left.addTransition(&lane_change_left);
    lane_change_left.addTransition(&keep_lane);

    // change lane right transitions
    lane_change_right.addTransition(&lane_change_right);
    lane_change_right.addTransition(&keep_lane);

    // // Set initial state
    state_ = &ready;

    cout << "constructor." <<  endl;
    transition();
}

Behavior::Behavior(const Behavior &obj) {
   cout << "Copy constructor allocating ptr." << endl;
   // ptr = new int;
   // *ptr = *obj.ptr; // copy the value
}

Behavior::~Behavior() {
    cout << "behavior gone" << endl;
}

void Behavior::transition() {

    vector<State*> states = state_->getTransitions();
    cout << "Current ID: " << state_->getId() << " Size: " << states.size() << endl ;
    for (int i = 0; i < states.size(); i++) {
        cout << "ID: " << states[i]->getId() << endl;
    }
}


