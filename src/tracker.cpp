/*
 * tracker.h
 *
 * this file has a class definition:
 *  - public
 *      - constructor tracker(nh, kf_parmas);
 *      - load_kf() {}
 *      - update_kf() { updates kalman filter) and other get methods
 *  - private has node handle
 *     - THIS ENABLES DIFFERENT THINGS TO TALK TO EACH OTHER.
 *     - void callback(msg);
 *     - kf;
 *     -
 *
 *
 *
 * tracker_node.cpp
 *
 * int main() {
 *
 *
 *  - create nh
 *  - kf_params is kf_params
 *  - create tracker node ( nh, kf_params)
 *  -
 *
 *
 *
 * tracker.cpp
 *
 *    tracker::tracker(nh, kf_params) : nh_(nh), kf_(kf_params) {
 *          // setup a subscriber which calls the callback
 *
 * }
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  -
