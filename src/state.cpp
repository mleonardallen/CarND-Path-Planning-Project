#include "state.h"

using namespace std;

State::State() {
}
State::~State() {}

void State::setId(State::StateId id) {
  id_ = id;
}

State::StateId State::getId() {
  return id_;
}

void State::addTransition(State::State *state) {
  transition_states_.push_back(state);
}

vector<State*> State::getTransitions() {
  return transition_states_;
}