--------------------------------------------------------------------------------
Project contribution guide
--------------------------------------------------------------------------------

Software version
- Make sure to check contributions with Arduido software v. 1.6.8 compiler

--------------------------------------------------------------------------------
Robot state documentation
--------------------------------------------------------------------------------

State 0:
About: Initial/wandering state
Task: Check general sensor readings

State 1:
About: At the corner of the goal zone
Task: Go a bit forwards, then rotate until facing the goal zone

State 2:
About: Currently heading towards goal zone
Task: Go fowards

State 3:
About: Currently backing out of the goal zone
Task: Go backwards for a small period of time