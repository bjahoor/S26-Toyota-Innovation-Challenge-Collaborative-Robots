# Reactive Safety Supervisor

High-level notes for a reactive safety supervisor that monitors the robot and
intervenes when conditions look unsafe.

## Idea
- Watch live robot state (position, speed, COM port link health)
- React to unsafe conditions (stop / slow / hold)

## Now
- [ ] Define what "unsafe" means (limits, faults, lost link)

## Next
- [ ] Hook supervisor into the control loop
- [ ] Trigger a safe stop on violation

## Later
- [ ] Logging of safety events
- [ ] Recovery after a stop
